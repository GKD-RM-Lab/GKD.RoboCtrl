#include "ctrl/robot.h"
#include "core/async.hpp"
#include "ctrl/shoot.h"
#include "device/controlpad.h"
#include "device/motor/dji.h"

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
    chassis_ = device::chassis_registry::current();
    gimbal_ = device::gimbal_registry::current();
    if (info.enable_shoot && !roboctrl::init(info.shoot_info)) {
        return false;
    }

    set_state(robot_state::NoForce);

    roboctrl::init(motion_control::info_type{
        .control_pad_key = control_pad_key_,
        .enable_shoot = enable_shoot_});
    roboctrl::spawn(task());

    log_info("Robot initiated");
    
    return true;
}

void robot::set_state(robot_state state) {
    state_ = state;
    if (state == robot_state::NoForce) {
    }
    const bool enabled = state != robot_state::NoForce;
    if (enable_chassis_ && chassis_) chassis_->set_enabled(enabled);
    if (enable_gimbal_ && gimbal_) gimbal_->set_enabled(enabled);
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
