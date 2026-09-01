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
        {"CAN_CHASSIS"},
        {"CAN_GIMBAL"}
    };

    constexpr std::initializer_list<io::serial::info_type> serials= {
        {"serial1","/dev/IMU_HERO",115200}
    };

    /// @brief 底盘电机共用的pid
    constexpr utils::linear_pid::params_type chassis_motor_pid = {
            .kp =           15000.0f,
            .ki =           10.0f,
            .kd =           0.0f,
            .max_out =      14000.0f,
            .max_iout =     2000.0f
    };

    constexpr utils::rad_pid::params_type gimbal_pid = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_out = 1.0f,
        .max_iout = 0.0f
    };

    constexpr std::initializer_list<device::dji_motor::info_type> dji_motors = {
        {device::dji_motor::M3508,3,"left_front_motor"  ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,4,"right_front_motor" ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,1,"right_rear_motor"  ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,2,"left_rear_motor"   ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M6020,1,"gimbal_yaw_motor"  ,"CAN_GIMBAL",1,chassis_motor_pid,2ms},
        {device::dji_motor::M6020,2,"gimbal_pitch_motor","CAN_GIMBAL",1,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,1,"left_friction"     ,"CAN_GIMBAL",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,2,"right_friction"    ,"CAN_GIMBAL",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M2006,3,"trigger"           ,"CAN_GIMBAL",0.075,chassis_motor_pid,2ms}
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
            .yaw_motor_params{"gimbal_yaw_motor", gimbal_pid},
            .init_yaw_motor_params{"gimbal_yaw_motor", gimbal_pid},
            .pitch_motor_params{"gimbal_pitch_motor", gimbal_pid}
        },
        .chassis_info{

        },
        .shoot_info{
            .friction_params{.acc = 100.0f},
            .friction_max_speed = 20.0f
        },
        .enable_chassis = true,
        .enable_gimbal = true,
        .enable_shoot = true
    };
}
