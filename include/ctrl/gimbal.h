/**
 * @file gimbal.h
 * @brief 云台控制器。
 * @details 将多个 PID 控制电机组合成云台系统，负责角度设定与姿态维护。
 */
#pragma once
#include <algorithm>
#include <chrono>
#include <string_view>
#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "device/motor/ref.hpp"
#include "utils/pid.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{
using namespace std::chrono_literals;
/**
 * @brief 云台主控制器单例。
 */
class gimbal : public utils::singleton_base<gimbal>,public logable<gimbal>{
public:
    inline std::string desc()const{return "gimbal";}
    inline fp32 yaw()const{return yaw_;}
    inline void set_yaw(fp32 yaw){yaw_ = yaw;}
    inline void add_yaw(fp32 delta){yaw_ = utils::rad_format(yaw_ + delta);}
    inline fp32 pitch()const{return pitch_;}
    inline void set_pitch(fp32 pitch){pitch_ = std::clamp(pitch, pitch_min_, pitch_max_);}
    inline void add_pitch(fp32 delta){set_pitch(pitch_ + delta);}

    /**
     * @brief 初始化参数。
     */
    struct info_type{
        using owner_type = gimbal;

        std::string_view imu_key {"imu"};
        std::string_view yaw_motor_key {"gimbal_yaw_motor"};
        std::string_view pitch_motor_key {"gimbal_pitch_motor"};
        utils::rad_pid::params_type yaw_angle_pid {};
        utils::rad_pid::params_type pitch_angle_pid {};
        fp32 yaw_direction {1.0f};
        fp32 pitch_direction {1.0f};
        fp32 pitch_min {-0.3f};
        fp32 pitch_max {0.3f};
        std::chrono::steady_clock::duration control_time {1ms};
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
    fp32 pitch_ = 0;
    fp32 pitch_min_ {-0.3f};
    fp32 pitch_max_ {0.3f};
    fp32 yaw_direction_ {1.0f};
    fp32 pitch_direction_ {1.0f};
    fp32 yaw_zero_ {};
    bool yaw_zero_initialized_ {false};
    bool targets_initialized_ {false};
    std::string_view imu_key_;
    std::chrono::steady_clock::duration control_time_ {1ms};
    utils::rad_pid yaw_angle_pid_;
    utils::rad_pid pitch_angle_pid_;
    device::motor_ref yaw_motor_;
    device::motor_ref pitch_motor_;
};

static_assert(utils::singleton<gimbal>);
}
