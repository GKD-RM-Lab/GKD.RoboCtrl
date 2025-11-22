/**
 * @file gimbal.h
 * @brief 云台控制器。
 * @details 将多个 PID 控制电机组合成云台系统，负责角度设定与姿态维护。
 */
#pragma once
#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/pid.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{
/**
 * @brief 云台主控制器单例。
 */
class gimbal : public utils::singleton_base<gimbal>,public logable<gimbal>{
public:
    inline std::string desc()const{return "gimbal";}
    inline fp32 yaw()const{return yaw_;}
    inline void set_yaw(fp32 yaw){yaw_ = yaw;}

    /**
     * @brief 初始化参数。
     */
    struct info_type{
        using owner_type = gimbal;

        utils::rad_pid_motor<device::dji_motor>::params_type yaw_motor_params;
        utils::rad_pid_motor<device::dji_motor>::params_type init_yaw_motor_params;
        utils::rad_pid_motor<device::dji_motor>::params_type pitch_motor_params;
    };

    /**
     * @brief 初始化云台资源与控制器。
     */
    bool init(const info_type& info);

    /**
     * @brief 主循环任务，负责刷新控制器。
     */
    awaitable<void> task();

private:
    fp32 yaw_ = 0;

    utils::rad_pid_motor<device::dji_motor> yaw_motor_;
    utils::rad_pid_motor<device::dji_motor> init_yaw_motor_; // 用于开机把云台转到初始位置
    utils::rad_pid_motor<device::dji_motor> pitch_motor_;
};

static_assert(utils::singleton<gimbal>);
}
