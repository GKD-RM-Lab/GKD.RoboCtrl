#include "ctrl/motion_control.h"

#include "core/multiton.hpp"
#include "ctrl/robot.h"
#include "ctrl/shoot.h"

namespace roboctrl::ctrl {

bool motion_control::init(const info_type& info) {
    control_pad_key_ = info.control_pad_key;
    control_time_ = info.control_time;
    enable_shoot_ = info.enable_shoot;
    auto& robot = roboctrl::get<ctrl::robot>();
    chassis_ = robot.chassis();
    gimbal_ = robot.gimbal();
    auto& pad = roboctrl::get<device::control_pad>(control_pad_key_);
    pad.on_update([this](const device::control_pad_state& input) { input_ = input; });
    roboctrl::spawn(task());
    return true;
}

void motion_control::stop_outputs() {
    if (chassis_) {
        chassis_->set_planar_velocity({0.0f, 0.0f});
        chassis_->set_rotate_speed(0.0f);
    }
    if (gimbal_) {
        gimbal_->set_target_yaw(0.0f);
        gimbal_->set_target_pitch(0.0f);
    }
    if (roboctrl::get<robot>().state() == robot_state::NoForce) {
        mapper_.reset();
    }
}

void motion_control::dispatch(const control_command& command) {
    if (chassis_) {
        chassis_->set_planar_velocity(command.velocity);
        chassis_->set_rotate_speed(command.rotate_speed);
    }
    if (gimbal_) {
        if (command.use_pitch_target) gimbal_->set_target_pitch(command.pitch_target);
        else gimbal_->add_pitch(command.pitch_delta);
        gimbal_->add_yaw(command.yaw_delta);
    }
    if (enable_shoot_ && roboctrl::get<robot>().state() != robot_state::NoForce) {
        auto& shoot = roboctrl::get<ctrl::shoot>();
        shoot.set_friction_enabled(command.friction_enabled);
        shoot.set_firing(command.firing);
    }
}

awaitable<void> motion_control::task() {
    while (true) {
        auto command = mapper_.update(input_);
        auto& robot = roboctrl::get<ctrl::robot>();
        if (command.arm_requested && robot.state() == robot_state::NoForce) {
            robot.set_state(robot_state::FollowGimbal);
        }
        if (robot.state() == robot_state::NoForce ||
            roboctrl::get<device::control_pad>(control_pad_key_).offline()) {
            stop_outputs();
        } else {
            dispatch(command);
        }
        co_await roboctrl::wait_for(control_time_);
    }
}

} // namespace roboctrl::ctrl
