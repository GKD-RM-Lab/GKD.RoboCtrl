#pragma once

#include <format>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>

#include "ctrl/robot.h"
#include "device/controlpad.h"
#include "device/imu/serial_imu.hpp"
#include "device/motor/dji.h"
#include "io/can.h"
#include "io/serial.h"

namespace roboctrl::config {

template<typename Info>
void validate_unique_keys(std::string_view kind, std::initializer_list<Info> infos) {
    std::unordered_set<typename Info::key_type> keys;
    for (const auto& info : infos) {
        const auto key = info.key();
        if constexpr (std::convertible_to<decltype(key), std::string_view>) {
            if (std::string_view{key}.empty()) {
                throw std::invalid_argument(std::format("{} key must not be empty", kind));
            }
        }
        if (!keys.emplace(key).second) {
            throw std::invalid_argument(std::format("duplicate {} key {}", kind, key));
        }
    }
}

inline void validate_configuration(
    std::initializer_list<io::can::info_type> cans,
    std::initializer_list<io::serial::info_type> serials,
    std::initializer_list<device::dji_motor::info_type> motors,
    const device::control_pad::info_type& control_pad,
    const device::serial_imu::info_type& imu,
    const ctrl::robot::info_type& robot)
{
    validate_unique_keys("CAN", cans);
    validate_unique_keys("serial", serials);
    validate_unique_keys("DJI motor", motors);

    std::unordered_set<std::string_view> can_names;
    for (const auto& can : cans) {
        can_names.emplace(can.key());
    }

    std::unordered_set<std::string_view> serial_names;
    for (const auto& serial : serials) {
        serial_names.emplace(serial.key());
    }

    if (!serial_names.contains(control_pad.serial_name)) {
        throw std::invalid_argument(std::format(
            "control pad references missing serial {}", control_pad.serial_name));
    }
    if (!serial_names.contains(imu.serial_name)) {
        throw std::invalid_argument(std::format(
            "IMU {} references missing serial {}", imu.name, imu.serial_name));
    }

    std::unordered_set<std::string_view> motor_names;
    std::unordered_set<std::string> feedback_slots;
    std::unordered_set<std::string> command_slots;
    for (const auto& motor : motors) {
        motor_names.emplace(motor.name);
        if (!can_names.contains(motor.can_name)) {
            throw std::invalid_argument(std::format(
                "DJI motor {} references missing CAN {}", motor.name, motor.can_name));
        }
        if (motor.id < 1 || motor.id > 8) {
            throw std::invalid_argument(std::format(
                "DJI motor {} has invalid id {}", motor.name, motor.id));
        }
        if (motor.radius <= 0.0f) {
            throw std::invalid_argument(std::format(
                "DJI motor {} has non-positive radius", motor.name));
        }
        if (motor.control_time <= std::chrono::steady_clock::duration::zero()) {
            throw std::invalid_argument(std::format(
                "DJI motor {} has non-positive control period", motor.name));
        }

        const int feedback_id = motor.type_ == device::dji_motor::M6020
            ? 0x204 + motor.id
            : 0x200 + motor.id;
        const auto feedback_slot = std::format("{}:{:x}", motor.can_name, feedback_id);
        if (!feedback_slots.emplace(feedback_slot).second) {
            throw std::invalid_argument(std::format(
                "duplicate DJI feedback slot {}", feedback_slot));
        }

        int command_id = 0;
        int command_index = 0;
        if (motor.type_ == device::dji_motor::M6020) {
            command_id = motor.id <= 4 ? 0x1ff : 0x2ff;
            command_index = motor.id <= 4 ? motor.id - 1 : motor.id - 5;
        } else {
            command_id = motor.id <= 4 ? 0x200 : 0x1ff;
            command_index = motor.id <= 4 ? motor.id - 1 : motor.id - 5;
        }
        const auto command_slot = std::format(
            "{}:{:x}:{}", motor.can_name, command_id, command_index);
        if (!command_slots.emplace(command_slot).second) {
            throw std::invalid_argument(std::format(
                "duplicate DJI command slot {}", command_slot));
        }
    }

    const auto require_motor = [&](std::string_view name, std::string_view subsystem) {
        if (!motor_names.contains(name)) {
            throw std::invalid_argument(std::format(
                "{} requires missing motor {}", subsystem, name));
        }
    };

    if (robot.enable_chassis) {
        require_motor("left_front_motor", "chassis");
        require_motor("right_front_motor", "chassis");
        require_motor("left_rear_motor", "chassis");
        require_motor("right_rear_motor", "chassis");
    }
    if (robot.enable_gimbal) {
        require_motor("gimbal_yaw_motor", "gimbal");
        require_motor("gimbal_pitch_motor", "gimbal");
    }
    if (robot.enable_shoot) {
        require_motor("left_friction", "shoot");
        require_motor("right_friction", "shoot");
        require_motor("trigger", "shoot");
    }
}

} // namespace roboctrl::config
