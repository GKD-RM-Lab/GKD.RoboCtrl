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
            co_await device::set_motor<device::dji_motor>("left_friction", 0);
            co_await device::set_motor<device::dji_motor>("right_friction", 0);
            co_await device::set_motor<device::dji_motor>("trigger", 0);
        }

        friction_ramp_.update(firing_ ? info_.friction_max_speed : .0f);

        co_await device::set_motor<device::dji_motor>("left_friction", -friction_ramp_.state());
        co_await device::set_motor<device::dji_motor>("right_friction", friction_ramp_.state());
        
        co_await roboctrl::wait_for(1ms);
    } 
}