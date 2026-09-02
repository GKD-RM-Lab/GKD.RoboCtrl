/**
 * @file robot.h
 * @brief 机器人级状态与子系统生命周期编排。
 */
#pragma once

#include <string>
#include <vector>

#include "core/async.hpp"
#include "ctrl/control_mapping.hpp"
#include "ctrl/motion_control.h"
#include "ctrl/shoot.h"
#include "device/chassis/base.hpp"
#include "device/controlpad.h"
#include "device/gimbal/base.hpp"
#include "device/motor/base.hpp"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{

/** @brief 机器人控制状态。 */
enum class robot_state{
    NoForce,
    FinishInit,
    FollowGimbal,
    Search,
    Idle,
    NotFollow
};

class robot : public utils::singleton_base<robot>,public logable<robot>{
public:
    robot() = default;
    /** @brief 机器人初始化参数及启用的子系统。 */
    struct info_type{
        using owner_type = robot;
        device::gimbal_base::info_type gimbal_info;
        device::chassis_base::info_type chassis_info;
        shoot::info_type shoot_info;
        /// 配置选择的具体底盘类；当前仅实现标准麦轮底盘。
        std::string chassis_type {"ctrl.standard_mecanum_chassis.v1"};
        /// 配置选择的具体云台类；当前仅实现标准双轴 IMU 云台。
        std::string gimbal_type {"ctrl.standard_imu_2axis_gimbal.v1"};
        std::string control_pad_key {"control_pad"};
        bool enable_chassis {true};
        bool enable_gimbal {false};
        bool enable_shoot {false};
    };

    /** @brief 创建具体底盘/云台并初始化控制子系统。 */
    bool init(const info_type& info);
    std::string desc()const{return "robot";}

    /** @brief 运行机器人级状态机任务。 */
    roboctrl::awaitable<void> task();
    /** Return the configured chassis, or nullptr when the chassis is disabled. */
    device::chassis_base* chassis() noexcept { return chassis_; }
    const device::chassis_base* chassis() const noexcept { return chassis_; }

    /** Return the configured gimbal, or nullptr when the gimbal is disabled. */
    device::gimbal_base* gimbal() noexcept { return gimbal_; }
    const device::gimbal_base* gimbal() const noexcept { return gimbal_; }

    /** @brief 获取当前机器人状态。 */
    robot_state state()const{return state_;}
    /** @brief 设置当前机器人状态并更新输出使能。 */
    void set_state(robot_state state);
private:
    robot_state state_ {robot_state::NoForce};
    std::string control_pad_key_;
    bool enable_chassis_ {false};
    bool enable_gimbal_ {false};
    bool enable_shoot_ {false};
    device::chassis_base* chassis_ {nullptr};
    device::gimbal_base* gimbal_ {nullptr};
    std::vector<device::motor_base*> controlled_motors_;
};

static_assert(utils::singleton<robot>);
}
