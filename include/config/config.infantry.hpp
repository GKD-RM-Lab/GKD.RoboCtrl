#pragma once

#include <string>
#include <vector>
#include <initializer_list>

#include "ctrl/robot.h"
#include "device/controlpad.h"
#include "device/imu/serial_imu.hpp"
#include "device/motor/m9025.h"
#include "io/can.h"
#include "io/serial.h"
#include "io/udp.h"
#include "device/motor/dji.h"
#include "utils/pid.h"

namespace roboctrl::config{
    constexpr std::initializer_list<io::can::info_type> cans = {
        {"can0"},
        {"can1"}
    };

    constexpr std::initializer_list<io::serial::info_type> serials= {
        {"serial1","/dev/IMU_HERO",115200}
    };

    constexpr utils::linear_pid::params_type motor_pid = {
        .kp = 15000.0f,
        .ki = 10.0f,
        .kd = 0.0f,
        .max_out = 14000.0f,
        .max_iout = 2000.0f
    };

    constexpr utils::rad_pid::params_type gimbal_pid = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_out = 1.0f,
        .max_iout = 0.0f
    };

    constexpr utils::rad_pid::params_type chassis_follow_pid = {
        .kp = 2.0f,
        .ki = 0.0f,
        .kd = 10.0f,
        .max_out = 6.0f,
        .max_iout = 0.2f
    };

    constexpr std::initializer_list<device::dji_motor::info_type> dji_motors = {
        {device::dji_motor::M3508,1,"left_front_motor","can1",0.075,motor_pid,2ms},
        {device::dji_motor::M3508,2,"right_front_motor","can1",0.075,motor_pid,2ms},
        {device::dji_motor::M3508,3,"right_rear_motor","can1",0.075,motor_pid,2ms},
        {device::dji_motor::M3508,4,"left_rear_motor","can1",0.075,motor_pid,2ms},
        {device::dji_motor::M6020,1,"gimbal_yaw_motor","can0",1,motor_pid,2ms},
        {device::dji_motor::M6020,2,"gimbal_pitch_motor","can0",1,motor_pid,2ms},
        {device::dji_motor::M3508,1,"left_friction","can0",0.075,motor_pid,2ms},
        {device::dji_motor::M3508,2,"right_friction","can0",0.075,motor_pid,2ms},
        {device::dji_motor::M2006,3,"trigger","can0",0.075,motor_pid,2ms}
    };

    constexpr device::control_pad::info_type control_pad{
        "serial1"
    };

    constexpr device::serial_imu::info_type imu{
        "imu",
        "serial1"
    };

    constexpr roboctrl::ctrl::robot::info_type robot{
        .gimbal_info{
            .imu_key = "imu",
            .yaw_motor_key = "gimbal_yaw_motor",
            .pitch_motor_key = "gimbal_pitch_motor",
            .yaw_angle_pid = gimbal_pid,
            .pitch_angle_pid = gimbal_pid,
            .control_time = 1ms
        },
        .chassis_info{
            .follow_pid = chassis_follow_pid,
            .follow_direction = -1.0f,
            .control_time = 2ms
        },
        .shoot_info{
            .friction_params{.acc = 100.0f},
            .friction_max_speed = 20.0f,
            .trigger_speed = 6.0f
        },
        .control_pad_key = "serial1",
        .enable_chassis = true,
        .enable_gimbal = true,
        .enable_shoot = true
    };
}
