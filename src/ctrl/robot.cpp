#include "ctrl/robot.h"
#include "core/async.hpp"
#include "ctrl/shoot.h"
#include "device/controlpad.h"
#include "device/motor/dji.h"

#include <algorithm>

using namespace roboctrl::ctrl;

bool robot::init(const info_type& info){
    control_pad_key_ = info.control_pad_key;
    enable_chassis_ = info.enable_chassis;
    enable_gimbal_ = info.enable_gimbal;
    enable_shoot_ = info.enable_shoot;

    if (info.enable_chassis && !device::chassis_registry::init(info.chassis_type, info.chassis_info)) {
        return false;
    }
    if (info.enable_gimbal && !device::gimbal_registry::init(info.gimbal_type, info.gimbal_info)) {
        return false;
    }
    chassis_ = info.enable_chassis ? device::chassis_registry::current() : nullptr;
    gimbal_ = info.enable_gimbal ? device::gimbal_registry::current() : nullptr;
    if (info.enable_shoot && !roboctrl::init(info.shoot_info)) {
        return false;
    }

    controlled_motors_.clear();
    const auto bind_motor = [this](const std::string& key) {
        auto* motor = &roboctrl::get<device::dji_motor>(key);
        if (std::find(controlled_motors_.begin(), controlled_motors_.end(), motor) ==
            controlled_motors_.end()) {
            controlled_motors_.push_back(motor);
        }
    };
    if (info.enable_chassis) {
        bind_motor(info.chassis_info.left_front_motor);
        bind_motor(info.chassis_info.right_front_motor);
        bind_motor(info.chassis_info.left_rear_motor);
        bind_motor(info.chassis_info.right_rear_motor);
    }
    if (info.enable_gimbal) {
        bind_motor(info.gimbal_info.yaw_motor_key);
        bind_motor(info.gimbal_info.pitch_motor_key);
    }
    if (info.enable_shoot) {
        bind_motor(info.shoot_info.left_friction_motor);
        bind_motor(info.shoot_info.right_friction_motor);
        bind_motor(info.shoot_info.trigger_motor);
    }

    set_state(robot_state::NoForce);

    if (!roboctrl::init(motion_control::info_type{
        .control_pad_key = control_pad_key_,
        .enable_shoot = enable_shoot_})) {
        return false;
    }
    roboctrl::spawn(task());

    log_info("Robot initiated");
    
    return true;
}

void robot::set_state(robot_state state) {
    if (state != robot_state::NoForce && roboctrl::async::shutdown_requested()) {
        log_warn("Ignored request to leave NoForce during shutdown");
        state = robot_state::NoForce;
    }
    state_ = state;
    if (state == robot_state::NoForce && enable_shoot_) {
        auto& shoot = roboctrl::get<ctrl::shoot>();
        shoot.set_firing(false);
        shoot.set_friction_enabled(false);
    }
    const bool enabled = state != robot_state::NoForce;
    if (enable_chassis_ && chassis_) chassis_->set_enabled(enabled);
    if (enable_gimbal_ && gimbal_) gimbal_->set_enabled(enabled);
    for (auto* motor : controlled_motors_) {
        if (motor) motor->set_enabled(enabled);
    }
}

roboctrl::awaitable<void> robot::task(){
    auto& control_pad = roboctrl::get<device::control_pad>(control_pad_key_);
    while (true) {
        if (state_ != robot_state::NoForce && control_pad.offline()) {
            log_warn("Control pad offline; entering NoForce");
            set_state(robot_state::NoForce);
        }
        co_await roboctrl::wait_for(10ms);
    }
}
