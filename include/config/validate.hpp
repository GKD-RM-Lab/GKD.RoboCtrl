#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>
#include <format>
#include <initializer_list>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "ctrl/robot.h"
#include "device/controlpad.h"
#include "device/imu/serial_imu.hpp"
#include "device/motor/dji.h"
#include "device/motor/j6006.h"
#include "device/motor/m9025.h"
#include "io/can.h"
#include "io/serial.h"

namespace roboctrl::config {

template<typename Info>
void validate_unique_keys(std::string_view kind, std::span<const Info> infos) {
    std::unordered_set<typename Info::key_type> keys;
    for (const auto& info : infos) {
        const auto key = info.key();
        if constexpr (std::convertible_to<decltype(key), std::string_view>) {
            if (std::string_view{key}.empty())
                throw std::invalid_argument(std::format("{} key must not be empty", kind));
        }
        if (!keys.emplace(key).second)
            throw std::invalid_argument(std::format("duplicate {} key {}", kind, key));
    }
}

template<typename Info>
void validate_unique_keys(std::string_view kind, std::initializer_list<Info> infos) {
    validate_unique_keys(kind, std::span{infos.begin(), infos.size()});
}

inline bool valid_pid(const auto& pid, float max_command = std::numeric_limits<float>::max()) {
    return std::isfinite(pid.kp) && std::isfinite(pid.ki) && std::isfinite(pid.kd) &&
        std::isfinite(pid.max_out) && std::isfinite(pid.max_iout) &&
        pid.max_out >= 0 && pid.max_iout >= 0 &&
        pid.max_out <= max_command && pid.max_iout <= max_command;
}

/** Hardware-free validation of owned info types, bus slots and actuator ownership. */
inline void validate_configuration(
    std::span<const io::can::info_type> cans,
    std::span<const io::serial::info_type> serials,
    std::span<const device::dji_motor::info_type> motors,
    const device::control_pad::info_type& control_pad,
    const device::serial_imu::info_type& imu,
    const ctrl::robot::info_type& robot,
    std::span<const device::serial_imu::info_type> additional_imus = {},
    std::span<const device::j6006::info_type> j6006_motors = {},
    std::span<const device::m9025::info_type> m9025_motors = {})
{
    const auto require = [](bool condition, const std::string& message) {
        if (!condition) throw std::invalid_argument(message);
    };
    validate_unique_keys("CAN", cans);
    validate_unique_keys("serial", serials);
    validate_unique_keys("DJI motor", motors);
    validate_unique_keys("J6006 motor", j6006_motors);
    validate_unique_keys("M9025 motor", m9025_motors);
    std::unordered_set<std::string> can_names, can_interfaces;
    std::unordered_map<std::string, bool> serial_names;
    std::unordered_set<std::string> serial_devices;
    for (const auto& can : cans) {
        require(!can.interface_name.empty(), "CAN interface_name must not be empty");
        require(can_interfaces.insert(can.interface_name).second, "multiple CAN keys reference the same physical interface");
        can_names.insert(can.name);
    }
    for (const auto& serial : serials) {
        require(!serial.device.empty() && serial.baud_rate > 0, "serial has invalid device or baud rate");
        require(serial_devices.insert(serial.device).second, "multiple serial instances reference the same physical device");
        serial_names.emplace(serial.name, serial.raw);
    }
    require(!control_pad.name.empty() && serial_names.contains(control_pad.serial_name),
            "control pad name or serial reference is invalid");
    require(!serial_names.at(control_pad.serial_name), "control pad requires a keyed serial");
    require(robot.control_pad_key == control_pad.key(), "robot references missing control pad");
    std::unordered_set<std::string> imu_names, imu_channels;
    const auto check_imu = [&](const auto& item) {
        require(!item.name.empty() && imu_names.insert(item.name).second, "duplicate or empty IMU key");
        require(serial_names.contains(item.serial_name) && !serial_names.at(item.serial_name),
                "IMU requires an existing keyed serial");
        require(imu_channels.insert(item.serial_name).second, "two IMUs occupy serial key 1 on same serial");
        const auto sign = [](float value) { return value == 1.f || value == -1.f; };
        require(sign(item.roll_sign) && sign(item.pitch_sign) && sign(item.yaw_sign) &&
                sign(item.roll_rate_sign) && sign(item.pitch_rate_sign) && sign(item.yaw_rate_sign) &&
                std::isfinite(item.gyro_scale) && item.gyro_scale > 0, "invalid IMU signs or gyro scale");
    };
    check_imu(imu);
    for (const auto& item : additional_imus) check_imu(item);

    std::unordered_map<std::string, std::string> motor_types;
    std::unordered_set<std::string> receive_slots, command_slots, command_ids;
    const auto add_motor = [&](const auto& motor, std::string driver) {
        require(!motor.name.empty() && motor_types.emplace(motor.name, driver).second,
                "motor names must be unique across driver types");
        require(can_names.contains(motor.can_name), "motor references missing CAN: " + motor.name);
        require(std::isfinite(motor.radius) && motor.radius > 0 &&
                motor.control_time > std::chrono::steady_clock::duration::zero(),
                "motor radius or period invalid: " + motor.name);
    };
    const auto rx = [&](const std::string& bus, unsigned id) {
        require(receive_slots.insert(std::format("{}:{}", bus, id)).second,
                "conflicting motor feedback CAN ID");
    };
    for (const auto& motor : motors) {
        add_motor(motor, "dji");
        require(motor.id >= 1 && motor.id <= device::dji_motor::max_device_id(motor.type_) &&
                (motor.type_ == device::dji_motor::M2006 || motor.type_ == device::dji_motor::M3508 ||
                 motor.type_ == device::dji_motor::M6020), "invalid DJI model/id");
        require(valid_pid(motor.pid_params, device::dji_motor::command_current_limit(motor.type_)), "invalid DJI PID");
        const bool gimbal = motor.type_ == device::dji_motor::M6020;
        rx(motor.can_name, (gimbal ? 0x204 : 0x200) + motor.id);
        const unsigned command = gimbal ? (motor.id <= 4 ? 0x1ff : 0x2ff) :
                                           (motor.id <= 4 ? 0x200 : 0x1ff);
        const auto slot = std::format("{}:{}:{}", motor.can_name, command, (motor.id - 1) % 4);
        require(command_slots.insert(slot).second, "conflicting DJI command slot");
        command_ids.insert(std::format("{}:{}", motor.can_name, command));
    }
    std::unordered_set<std::string> j6006_receive_slots, j6006_controller_slots;
    for (const auto& motor : j6006_motors) {
        add_motor(motor, "j6006");
        require(device::j6006::valid_configuration(motor), "invalid J6006 configuration");
        const auto slot = std::format("{}:{}", motor.can_name, motor.master_id);
        // J6006 feedback multiplexes distinct controller IDs on one master ID.
        require(!receive_slots.contains(slot) || j6006_receive_slots.contains(slot),
                "J6006 feedback overlaps another motor protocol");
        receive_slots.insert(slot);
        j6006_receive_slots.insert(slot);
        require(j6006_controller_slots.insert(std::format("{}:{}", motor.can_name, motor.id)).second,
                "duplicate J6006 controller ID on bus");
        require(command_ids.insert(std::format("{}:{}", motor.can_name, 0x200 + motor.id)).second,
                "J6006 command conflicts with another motor protocol");
    }
    for (const auto& motor : m9025_motors) {
        add_motor(motor, "m9025");
        require(device::m9025::valid_configuration(motor), "invalid M9025 configuration");
        rx(motor.can_name, 0x140 + motor.id);
        require(command_ids.insert(std::format("{}:{}", motor.can_name, 0x140 + motor.id)).second,
                "M9025 command conflicts with another motor protocol");
    }
    // A command must not be interpreted as another device's feedback on the bus.
    for (const auto& motor : j6006_motors) {
        require(!command_ids.contains(std::format("{}:{}", motor.can_name, motor.master_id)),
                "J6006 feedback overlaps a motor command ID");
        require(!receive_slots.contains(std::format("{}:{}", motor.can_name, 0x200 + motor.id)),
                "J6006 command overlaps a motor feedback ID");
    }

    std::unordered_set<std::string> controlled_motors;
    const auto bind_motor = [&](const std::string& name, const std::string& type) {
        require(motor_types.contains(name) && motor_types.at(name) == type,
                "missing motor or driver mismatch: " + name);
        require(controlled_motors.insert(name).second, "actuator has multiple controllers: " + name);
    };
    if (robot.enable_chassis) {
        for (int direction : robot.chassis_info.wheel_directions)
            require(direction == 1 || direction == -1, "wheel directions must be +1 or -1");
        for (const auto* name : {&robot.chassis_info.left_front_motor, &robot.chassis_info.right_front_motor,
                                &robot.chassis_info.left_rear_motor, &robot.chassis_info.right_rear_motor})
            bind_motor(*name, "dji");
        require(robot.chassis_info.control_time > std::chrono::steady_clock::duration::zero() &&
                std::isfinite(robot.chassis_info.max_rotate_speed) && robot.chassis_info.max_rotate_speed > 0,
                "chassis has invalid control parameters");
    }
    const auto check_gimbal = [&](const auto& gimbal) {
        bind_motor(gimbal.yaw_motor_key, gimbal.yaw_motor_type);
        if (!gimbal.yaw_only) bind_motor(gimbal.pitch_motor_key, gimbal.pitch_motor_type);
        require(imu_names.contains(gimbal.imu_key), "gimbal references missing IMU");
        require(!(gimbal.yaw_current_control && gimbal.yaw_motor_type == "j6006") &&
                !(!gimbal.yaw_only && gimbal.pitch_current_control && gimbal.pitch_motor_type == "j6006"),
                "J6006 firmware speed mode cannot accept a direct current target");
        require(gimbal.control_time > std::chrono::steady_clock::duration::zero() &&
                gimbal.init_settle_time >= std::chrono::steady_clock::duration::zero() &&
                std::isfinite(gimbal.yaw_zero) && std::isfinite(gimbal.init_tolerance) && gimbal.init_tolerance > 0 &&
                std::isfinite(gimbal.init_pitch) && std::isfinite(gimbal.pitch_min) && std::isfinite(gimbal.pitch_max) &&
                gimbal.pitch_min <= gimbal.pitch_max && gimbal.init_pitch >= gimbal.pitch_min && gimbal.init_pitch <= gimbal.pitch_max &&
                (gimbal.yaw_direction == 1 || gimbal.yaw_direction == -1) &&
                (gimbal.yaw_angle_direction == 1 || gimbal.yaw_angle_direction == -1) &&
                (gimbal.yaw_recenter_direction == 1 || gimbal.yaw_recenter_direction == -1) &&
                (gimbal.pitch_direction == 1 || gimbal.pitch_direction == -1) &&
                valid_pid(gimbal.yaw_angle_pid) && valid_pid(gimbal.pitch_angle_pid) && valid_pid(gimbal.yaw_relative_pid) &&
                valid_pid(gimbal.yaw_rate_pid, 32767) && valid_pid(gimbal.pitch_rate_pid, 32767),
                "gimbal has invalid control parameters");
    };
    require(robot.enable_gimbal || (!robot.secondary_gimbal_info && !robot.large_yaw_info),
            "additional gimbals require enable_gimbal");
    if (robot.enable_gimbal) {
        check_gimbal(robot.gimbal_info);
        if (robot.secondary_gimbal_info) check_gimbal(*robot.secondary_gimbal_info);
        if (robot.large_yaw_info) check_gimbal(*robot.large_yaw_info);
    }
    const auto check_shoot = [&](const auto& shoot) {
        bind_motor(shoot.left_friction_motor, "dji");
        bind_motor(shoot.right_friction_motor, "dji");
        bind_motor(shoot.trigger_motor, "dji");
        require(shoot.control_time > std::chrono::steady_clock::duration::zero() &&
                shoot.jam_release_time >= std::chrono::steady_clock::duration::zero() &&
                std::isfinite(shoot.friction_params.acc) && shoot.friction_params.acc >= 0 &&
                std::isfinite(shoot.friction_max_speed) && shoot.friction_max_speed >= 0 &&
                std::isfinite(shoot.trigger_speed) && std::isfinite(shoot.friction_ready_speed) && shoot.friction_ready_speed > 0 && shoot.friction_ready_speed <= shoot.friction_max_speed &&
                std::isfinite(shoot.jam_current) && shoot.jam_current >= 0 &&
                std::isfinite(shoot.jam_speed) && shoot.jam_speed >= 0 &&
                (shoot.bullet_caliber == 17 || shoot.bullet_caliber == 42), "shoot has invalid control parameters");
    };
    require(!robot.secondary_shoot_info || (robot.enable_shoot && robot.secondary_gimbal_info),
            "secondary shoot requires enabled shoot and secondary gimbal");
    if (robot.enable_shoot) {
        check_shoot(robot.shoot_info);
        if (robot.secondary_shoot_info) check_shoot(*robot.secondary_shoot_info);
    }
    const auto& motion = robot.motion_info;
    require(motion.control_time > std::chrono::steady_clock::duration::zero() && valid_pid(motion.follow_pid) &&
            (motion.follow_direction == 1 || motion.follow_direction == -1) &&
            std::isfinite(motion.spin_recenter_speed) && motion.spin_recenter_speed >= 0 &&
            std::isfinite(motion.follow_tolerance) && motion.follow_tolerance > 0 &&
            std::isfinite(motion.search_yaw_speed) && std::isfinite(motion.search_pitch_speed) &&
            std::isfinite(motion.search_pitch_amplitude) && motion.search_pitch_amplitude >= 0 &&
            std::isfinite(motion.search_pitch_center), "invalid motion control parameters");
}

inline void validate_configuration(
    std::initializer_list<io::can::info_type> cans,
    std::initializer_list<io::serial::info_type> serials,
    std::initializer_list<device::dji_motor::info_type> motors,
    const device::control_pad::info_type& control_pad,
    const device::serial_imu::info_type& imu,
    const ctrl::robot::info_type& robot)
{
    validate_configuration(std::span{cans.begin(), cans.size()}, std::span{serials.begin(), serials.size()},
                           std::span{motors.begin(), motors.size()}, control_pad, imu, robot);
}

} // namespace roboctrl::config
