#pragma once

#include <concepts>
#include <cmath>
#include <cstdint>
#include <format>
#include <initializer_list>
#include <stdexcept>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <limits>

#include "ctrl/robot.h"
#include "device/controlpad.h"
#include "device/imu/serial_imu.hpp"
#include "device/motor/dji.h"
#include "io/can.h"
#include "io/serial.h"

namespace roboctrl::config {

/**
 * @brief 检查一组 info_type 的 key 非空且唯一。
 * @tparam Info 具备 `key_type` 和 `key()` 的配置类型。
 * @param kind 错误信息中显示的对象类型。
 * @param infos 待检查的配置集合。
 * @throws std::invalid_argument key 为空或重复时抛出。
 */
template<typename Info>
void validate_unique_keys(std::string_view kind, std::span<const Info> infos) {
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

template<typename Info>
void validate_unique_keys(std::string_view kind, std::initializer_list<Info> infos) {
    validate_unique_keys(kind, std::span{infos.begin(), infos.size()});
}

/**
 * @brief 校验所有设备、控制器及其跨对象引用。
 * @details 该函数只进行硬件访问前的静态校验，不打开 CAN、串口或其他设备。
 * @throws std::invalid_argument 任意 key、引用、ID、槽位或控制参数非法时抛出。
 */
inline void validate_configuration(
    std::span<const io::can::info_type> cans,
    std::span<const io::serial::info_type> serials,
    std::span<const device::dji_motor::info_type> motors,
    const device::control_pad::info_type& control_pad,
    const device::serial_imu::info_type& imu,
    const ctrl::robot::info_type& robot)
{
    validate_unique_keys("CAN", cans);
    validate_unique_keys("serial", serials);
    validate_unique_keys("DJI motor", motors);

    for (const auto& can : cans) {
        if (can.interface_name.empty()) {
            throw std::invalid_argument(std::format(
                "CAN {} interface_name must not be empty", can.name));
        }
    }
    for (const auto& serial : serials) {
        if (serial.device.empty() || serial.baud_rate == 0) {
            throw std::invalid_argument(std::format(
                "serial {} has invalid device or baud rate", serial.name));
        }
    }
    if (control_pad.name.empty() || control_pad.serial_name.empty()) {
        throw std::invalid_argument("control pad name and serial_name must not be empty");
    }
    if (imu.name.empty() || imu.serial_name.empty()) {
        throw std::invalid_argument("IMU name and serial_name must not be empty");
    }
    if (robot.control_pad_key.empty()) {
        throw std::invalid_argument("robot control_pad_key must not be empty");
    }

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
    if (robot.control_pad_key != control_pad.key()) {
        throw std::invalid_argument(std::format(
            "robot references missing control pad {}", robot.control_pad_key));
    }

    std::unordered_set<std::string_view> motor_names;
    std::unordered_set<std::string> feedback_slots;
    std::unordered_set<std::string> command_slots;
    const auto valid_pid = [](const auto& pid) {
        return std::isfinite(pid.kp) && std::isfinite(pid.ki) &&
            std::isfinite(pid.kd) && std::isfinite(pid.max_out) &&
            std::isfinite(pid.max_iout) && pid.max_out >= 0.0f &&
            pid.max_iout >= 0.0f;
    };
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
        const auto finite = [](float value) { return std::isfinite(value); };
        if (!finite(motor.radius) || !finite(motor.pid_params.kp) ||
            !finite(motor.pid_params.ki) || !finite(motor.pid_params.kd) ||
            !finite(motor.pid_params.max_out) || !finite(motor.pid_params.max_iout) ||
            motor.pid_params.max_out < 0.0f || motor.pid_params.max_iout < 0.0f ||
            motor.pid_params.max_out > static_cast<float>(std::numeric_limits<int16_t>::max()) ||
            motor.pid_params.max_iout > static_cast<float>(std::numeric_limits<int16_t>::max())) {
            throw std::invalid_argument(std::format(
                "DJI motor {} has invalid PID parameters", motor.name));
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
        require_motor(robot.chassis_info.left_front_motor, "chassis");
        require_motor(robot.chassis_info.right_front_motor, "chassis");
        require_motor(robot.chassis_info.left_rear_motor, "chassis");
        require_motor(robot.chassis_info.right_rear_motor, "chassis");
        if (robot.chassis_info.control_time <= std::chrono::steady_clock::duration::zero() ||
            !std::isfinite(robot.chassis_info.max_rotate_speed) ||
            robot.chassis_info.max_rotate_speed <= 0.0f) {
            throw std::invalid_argument("chassis has invalid control parameters");
        }
    }
    if (robot.enable_gimbal) {
        require_motor(robot.gimbal_info.yaw_motor_key, "gimbal");
        require_motor(robot.gimbal_info.pitch_motor_key, "gimbal");
        if (robot.gimbal_info.imu_key != imu.key()) {
            throw std::invalid_argument(std::format(
                "gimbal references missing IMU {}", robot.gimbal_info.imu_key));
        }
        if (robot.gimbal_info.control_time <= std::chrono::steady_clock::duration::zero() ||
            !std::isfinite(robot.gimbal_info.yaw_direction) ||
            !std::isfinite(robot.gimbal_info.pitch_direction) ||
            !std::isfinite(robot.gimbal_info.pitch_min) ||
            !std::isfinite(robot.gimbal_info.pitch_max) ||
            !valid_pid(robot.gimbal_info.yaw_angle_pid) ||
            !valid_pid(robot.gimbal_info.pitch_angle_pid) ||
            robot.gimbal_info.yaw_direction == 0.0f ||
            robot.gimbal_info.pitch_direction == 0.0f ||
            robot.gimbal_info.pitch_min > robot.gimbal_info.pitch_max) {
            throw std::invalid_argument("gimbal has invalid control parameters");
        }
    }
    if (robot.enable_shoot) {
        require_motor(robot.shoot_info.left_friction_motor, "shoot");
        require_motor(robot.shoot_info.right_friction_motor, "shoot");
        require_motor(robot.shoot_info.trigger_motor, "shoot");
        if (robot.shoot_info.control_time <= std::chrono::steady_clock::duration::zero() ||
            robot.shoot_info.jam_release_time < std::chrono::steady_clock::duration::zero() ||
            !std::isfinite(robot.shoot_info.friction_params.acc) ||
            robot.shoot_info.friction_params.acc < 0.0f ||
            !std::isfinite(robot.shoot_info.friction_max_speed) ||
            !std::isfinite(robot.shoot_info.trigger_speed) ||
            !std::isfinite(robot.shoot_info.friction_ready_speed) ||
            !std::isfinite(robot.shoot_info.jam_current) ||
            !std::isfinite(robot.shoot_info.jam_speed) ||
            robot.shoot_info.friction_max_speed < 0.0f ||
            robot.shoot_info.friction_ready_speed < 0.0f ||
            robot.shoot_info.jam_current < 0.0f ||
            robot.shoot_info.jam_speed < 0.0f) {
            throw std::invalid_argument("shoot has invalid control parameters");
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
    validate_configuration(
        std::span{cans.begin(), cans.size()},
        std::span{serials.begin(), serials.size()},
        std::span{motors.begin(), motors.size()},
        control_pad,
        imu,
        robot);
}

} // namespace roboctrl::config
