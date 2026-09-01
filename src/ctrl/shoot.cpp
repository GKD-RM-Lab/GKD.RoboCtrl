#include "ctrl/shoot.h"
#include "core/async.hpp"
#include "ctrl/robot.h"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/ramp.hpp"

using namespace roboctrl::ctrl;

bool shoot::init(const shoot::info_type& info)
{
    info_ = info;
    friction_ramp_ = utils::ramp_f{info_.friction_params};
    left_friction_motor_ = device::motor_ref::from<device::dji_motor>("left_friction");
    right_friction_motor_ = device::motor_ref::from<device::dji_motor>("right_friction");
    trigger_motor_ = device::motor_ref::from<device::dji_motor>("trigger");
    log_info("Shoot initiated");

    roboctrl::spawn(task());
    
    return true;
}

void shoot::set_firing(bool state)
{
    firing_ = state;
    log_info("set firing to {}",state);
}

roboctrl::awaitable<void> shoot::task()
{
    while(true){
        if(roboctrl::get<robot>().state() == robot_state::NoForce){
            friction_ramp_.reset();
            co_await left_friction_motor_.set(0);
            co_await right_friction_motor_.set(0);
            co_await trigger_motor_.set(0);
            co_await roboctrl::wait_for(1ms);
            continue;
        }

        friction_ramp_.update(firing_ ? info_.friction_max_speed : .0f);

        co_await left_friction_motor_.set(-friction_ramp_.state());
        co_await right_friction_motor_.set(friction_ramp_.state());
        
        co_await roboctrl::wait_for(1ms);
    } 
}
