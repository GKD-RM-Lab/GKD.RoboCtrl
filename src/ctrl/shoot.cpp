#include "ctrl/shoot.h"
#include "ctrl/shoot_logic.hpp"
#include "core/async.hpp"
#include "ctrl/robot.h"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/ramp.hpp"
#include <cmath>

using namespace roboctrl::ctrl;

bool shoot::init(const shoot::info_type& info)
{
    info_ = info;
    friction_ramp_ = utils::ramp_f{info_.friction_params};
    left_friction_motor_ = &roboctrl::get<device::dji_motor>(info_.left_friction_motor);
    right_friction_motor_ = &roboctrl::get<device::dji_motor>(info_.right_friction_motor);
    trigger_motor_ = &roboctrl::get<device::dji_motor>(info_.trigger_motor);
    log_info("Shoot initiated");

    roboctrl::spawn(task());
    
    return true;
}

void shoot::set_firing(bool state)
{
    if (firing_ == state) {
        return;
    }
    firing_ = state;
    log_info("set firing to {}",state);
}

void shoot::set_friction_enabled(bool state)
{
    if (friction_enabled_ == state) {
        return;
    }
    friction_enabled_ = state;
    if (!state) {
        firing_ = false;
    }
    log_info("set friction to {}", state);
}

bool shoot::friction_ready() const
{
    return std::fabs(left_friction_motor_->linear_speed()) > info_.friction_ready_speed &&
        std::fabs(right_friction_motor_->linear_speed()) > info_.friction_ready_speed;
}

roboctrl::awaitable<void> shoot::task()
{
    while(true){
        if(roboctrl::get<robot>().state() == robot_state::NoForce){
            friction_ramp_.reset();
            co_await left_friction_motor_->set(0);
            co_await right_friction_motor_->set(0);
            co_await trigger_motor_->set(0);
            jam_release_at_ = {};
            co_await roboctrl::wait_for(info_.control_time);
            continue;
        }

        const fp32 dt = std::chrono::duration_cast<std::chrono::duration<fp32>>(info_.control_time).count();
        friction_ramp_.update(friction_enabled_ ? info_.friction_max_speed : .0f, dt);

        co_await left_friction_motor_->set(-friction_ramp_.state());
        co_await right_friction_motor_->set(friction_ramp_.state());

        const auto now = std::chrono::steady_clock::now();
        const bool friction_is_ready = friction_ready();
        const bool motors_online = !left_friction_motor_->offline() &&
            !right_friction_motor_->offline() && !trigger_motor_->offline();
        const bool jammed = firing_ && friction_enabled_ && friction_is_ready && motors_online &&
            trigger_jammed(
                trigger_motor_->torque(), trigger_motor_->rpm(), info_.jam_current, info_.jam_speed);
        if (jammed && now >= jam_release_at_) {
            jam_release_at_ = now + info_.jam_release_time;
            log_warn("Trigger jam detected; pausing feed");
        }

        const bool can_feed = trigger_feed_allowed(
            firing_, friction_enabled_, friction_is_ready, motors_online, now < jam_release_at_);
        co_await trigger_motor_->set(can_feed ? info_.trigger_speed : 0.0f);
        
        co_await roboctrl::wait_for(info_.control_time);
    } 
}
