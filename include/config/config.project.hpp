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
        {"CAN_CHASSIS"}
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

    constexpr std::initializer_list<device::dji_motor::info_type> dji_motors = {
        {device::dji_motor::M3508,2,"left_front_motor"  ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,1,"right_front_motor" ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,4,"right_rear_motor"  ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
        {device::dji_motor::M3508,3,"left_rear_motor"   ,"CAN_CHASSIS",0.075,chassis_motor_pid,2ms},
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

        },
        .chassis_info{

        },
        .shoot_info{

        }
    };
}