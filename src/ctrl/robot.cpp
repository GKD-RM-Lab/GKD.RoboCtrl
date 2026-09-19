#include "ctrl/robot.h"
#include "core/async.hpp"
#include "ctrl/shoot.h"
#include "device/controlpad.h"
#include "device/motor/dji.h"

#include <algorithm>

using namespace roboctrl::ctrl;

bool robot::init(const info_type& info) {
    control_pad_key_ = info.control_pad_key;
    enable_chassis_ = info.enable_chassis;
    enable_gimbal_ = info.enable_gimbal;
    enable_shoot_ = info.enable_shoot;
    if (enable_chassis_ && !device::chassis_registry::init(info.chassis_type, info.chassis_info))
        return false;
    if (enable_gimbal_ && !device::gimbal_registry::init(info.gimbal_type, info.gimbal_info))
        return false;
    chassis_ = enable_chassis_ ? device::chassis_registry::current() : nullptr;
    gimbal_ = enable_gimbal_ ? device::gimbal_registry::current() : nullptr;
    if (enable_gimbal_ && info.secondary_gimbal_info) {
        secondary_gimbal_ = device::gimbal_registry::create(info.gimbal_type, *info.secondary_gimbal_info);
        if (!secondary_gimbal_) return false;
    }
    if (enable_gimbal_ && info.large_yaw_info) {
        large_yaw_ = device::gimbal_registry::create(info.gimbal_type, *info.large_yaw_info);
        if (!large_yaw_) return false;
    }
    if (enable_shoot_ && !roboctrl::init(info.shoot_info)) return false;
    if (enable_shoot_ && info.secondary_shoot_info) {
        secondary_shoot_ = std::make_unique<shoot>();
        if (!secondary_shoot_->init(*info.secondary_shoot_info)) return false;
    }
    controlled_motors_.clear();
    const auto bind_motor = [this](const std::string& key) {
        auto* motor = &roboctrl::get<device::dji_motor>(key);
        if (std::find(controlled_motors_.begin(), controlled_motors_.end(), motor) == controlled_motors_.end())
            controlled_motors_.push_back(motor);
    };
    if (enable_chassis_) {
        bind_motor(info.chassis_info.left_front_motor);
        bind_motor(info.chassis_info.right_front_motor);
        bind_motor(info.chassis_info.left_rear_motor);
        bind_motor(info.chassis_info.right_rear_motor);
    }
    set_state(robot_state::NoForce);
    for (auto* gimbal : {gimbal_, secondary_gimbal_, large_yaw_})
        if (gimbal) gimbal->start();
    if (enable_shoot_) roboctrl::get<shoot>().start();
    if (secondary_shoot_) secondary_shoot_->start();
    auto motion = info.motion_info;
    motion.control_pad_key = control_pad_key_;
    motion.enable_shoot = enable_shoot_;
    if (!roboctrl::init(motion)) return false;
    roboctrl::spawn(task());
    log_info("Robot initiated");
    return true;
}

bool robot::gimbals_initialized() const {
    return (!gimbal_ || gimbal_->initialized()) &&
        (!secondary_gimbal_ || secondary_gimbal_->initialized()) &&
        (!large_yaw_ || large_yaw_->initialized());
}

bool robot::gimbals_online() const {
    return (!gimbal_ || gimbal_->online()) &&
        (!secondary_gimbal_ || secondary_gimbal_->online()) &&
        (!large_yaw_ || large_yaw_->online());
}

void robot::set_state(robot_state state) {
    if (state != robot_state::NoForce && roboctrl::async::shutdown_requested()) {
        log_warn("Ignored request to leave NoForce during shutdown");
        state = robot_state::NoForce;
    }
    state_ = state;
    const bool gimbal_enabled = state != robot_state::NoForce;
    const bool motion_enabled = gimbal_enabled && state != robot_state::FinishInit;
    if (chassis_) chassis_->set_enabled(motion_enabled);
    for (auto* gimbal : {gimbal_, secondary_gimbal_, large_yaw_}) {
        if (!gimbal) continue;
        gimbal->set_enabled(gimbal_enabled);
        gimbal->set_recentering(state == robot_state::FinishInit);
    }
    for (auto* motor : controlled_motors_) motor->set_enabled(motion_enabled);
    if (enable_shoot_) roboctrl::get<shoot>().set_enabled(motion_enabled);
    if (secondary_shoot_) secondary_shoot_->set_enabled(motion_enabled);
}

roboctrl::awaitable<void> robot::task() {
    auto& control_pad = roboctrl::get<device::control_pad>(control_pad_key_);
    while (true) {
        if (state_ != robot_state::NoForce && control_pad.offline()) {
            log_warn("Control pad offline; entering NoForce");
            set_state(robot_state::NoForce);
        }
        co_await roboctrl::wait_for(10ms);
    }
}
