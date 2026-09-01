#include "ctrl/robot.h"
#include "core/async.hpp"
#include "ctrl/chassis.h"
#include "ctrl/gimbal.h"
#include "ctrl/shoot.h"
#include "device/motor/dji.h"

using namespace roboctrl::ctrl;

bool robot::init(const info_type& info){
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

    log_info("Robot initiated");
    
    return true;
}

void robot::set_state(robot_state state) {
    state_ = state;
    const bool enabled = state != robot_state::NoForce;
    roboctrl::for_each_instance<device::dji_motor>([enabled](device::dji_motor& motor) {
        motor.set_enabled(enabled);
    });
}

roboctrl::awaitable<void> robot::task(){
    co_return;
}
