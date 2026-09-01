#include "ctrl/robot.h"
#include "core/async.hpp"
#include "ctrl/chassis.h"
#include "ctrl/gimbal.h"
#include "ctrl/shoot.h"
#include "device/controlpad.h"
#include "device/motor/dji.h"

using namespace roboctrl::ctrl;

bool robot::init(const info_type& info){
    control_pad_key_ = info.control_pad_key;
    enable_chassis_ = info.enable_chassis;
    enable_gimbal_ = info.enable_gimbal;
    enable_shoot_ = info.enable_shoot;

    if (info.enable_chassis && !roboctrl::init(info.chassis_info)) {
        return false;
    }
    if (info.enable_gimbal && !roboctrl::init(info.gimbal_info)) {
        return false;
    }
    if (info.enable_shoot && !roboctrl::init(info.shoot_info)) {
        return false;
    }

    set_state(robot_state::NoForce);

    roboctrl::get<device::control_pad>(control_pad_key_).on_update(
        [this](const device::control_pad_state& input) { handle_control(input); });
    roboctrl::spawn(task());

    log_info("Robot initiated");
    
    return true;
}

void robot::handle_control(const device::control_pad_state& input) {
    const auto command = control_mapper_.update(input);
    if (command.arm_requested && state_ == robot_state::NoForce) {
        set_state(robot_state::FollowGimbal);
        log_info("Control pad armed robot");
    }
    if (state_ == robot_state::NoForce) {
        return;
    }

    if (enable_chassis_) {
        roboctrl::get<chassis>().set_velocity(command.velocity);
        roboctrl::get<chassis>().set_rotate_speed(command.rotate_speed);
    }
    if (enable_gimbal_) {
        auto& gimbal = roboctrl::get<ctrl::gimbal>();
        gimbal.add_yaw(command.yaw_delta);
        if (command.use_pitch_target) {
            gimbal.set_pitch(command.pitch_target);
        } else {
            gimbal.add_pitch(command.pitch_delta);
        }
    }
    if (enable_shoot_) {
        auto& shoot = roboctrl::get<ctrl::shoot>();
        shoot.set_friction_enabled(command.friction_enabled);
        shoot.set_firing(command.firing);
    }
}

void robot::set_state(robot_state state) {
    state_ = state;
    if (state == robot_state::NoForce) {
        control_mapper_.reset();
    }
    const bool enabled = state != robot_state::NoForce;
    roboctrl::for_each_instance<device::dji_motor>([enabled](device::dji_motor& motor) {
        motor.set_enabled(enabled);
    });
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
