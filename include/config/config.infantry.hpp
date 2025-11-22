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

namespace roboctrl::config{
    constexpr std::initializer_list<io::can::info_type> cans = {
        {"can0"},
        {"can1"}
    };

    constexpr std::initializer_list<io::serial::info_type> serials= {
        {"serial1","/dev/IMU_HERO",115200}
    };

    constexpr std::initializer_list<device::dji_motor::info_type> dji_motors = {
        {device::dji_motor::M3508,1,"","can1",0.075},
        {device::dji_motor::M3508,2,"","can1",0.075},
        {device::dji_motor::M3508,3,"","can1",0.075},
        {device::dji_motor::M3508,4,"","can1",0.075},
        {device::dji_motor::M6020,1,"gimbal_yaw_motor","can0",1},
        {device::dji_motor::M6020,2,"gimbal_pitch_motor","can0",1},
        {device::dji_motor::M3508,1,"left_friction","can0",0.075},
        {device::dji_motor::M3508,2,"right_friction","can0",0.075},
        {device::dji_motor::M2006,3,"trigger","can0",0.075}
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