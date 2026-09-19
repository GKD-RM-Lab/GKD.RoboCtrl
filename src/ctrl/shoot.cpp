#include "ctrl/shoot.h"
#include "core/async.hpp"
#include "ctrl/robot.h"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/ramp.hpp"
#include <cmath>

using namespace roboctrl::ctrl;

namespace roboctrl::ctrl::detail {

bool trigger_jammed(
    roboctrl::fp32 feedback_current,
    roboctrl::fp32 feedback_rpm,
    roboctrl::fp32 current_threshold,
    roboctrl::fp32 speed_threshold)
{
    return std::fabs(feedback_current) > std::fabs(current_threshold) &&
        std::fabs(feedback_rpm) < std::fabs(speed_threshold);
}

bool trigger_feed_allowed(
    bool firing,
    bool friction_enabled,
    bool friction_ready,
    bool motors_online,
    bool jam_hold_active)
{
    return firing && friction_enabled && friction_ready && motors_online && !jam_hold_active;
}

} // namespace roboctrl::ctrl::detail

#ifndef ROBOCTRL_TESTING

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
    std::chrono::steady_clock::time_point last_update_at {};
    while(true){
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = now - last_update_at;
        const bool valid_elapsed = last_update_at != std::chrono::steady_clock::time_point{} &&
            elapsed > std::chrono::steady_clock::duration::zero() &&
            elapsed <= info_.control_time * 5;
        const fp32 dt = valid_elapsed
            ? std::chrono::duration_cast<std::chrono::duration<fp32>>(elapsed).count()
            : 0.0f;
        last_update_at = now;

        if(roboctrl::get<robot>().state() == robot_state::NoForce){
            friction_ramp_.reset();
            co_await left_friction_motor_->set(0);
            co_await right_friction_motor_->set(0);
            co_await trigger_motor_->set(0);
            jam_release_at_ = {};
            co_await roboctrl::wait_for(info_.control_time);
            continue;
        }

        friction_ramp_.update(friction_enabled_ ? info_.friction_max_speed : .0f, dt);

        co_await left_friction_motor_->set(-friction_ramp_.state());
        co_await right_friction_motor_->set(friction_ramp_.state());

        const bool friction_is_ready = friction_ready();
        const bool motors_online = !left_friction_motor_->offline() &&
            !right_friction_motor_->offline() && !trigger_motor_->offline();
        const bool jammed = firing_ && friction_enabled_ && friction_is_ready && motors_online &&
            detail::trigger_jammed(
                trigger_motor_->torque(), trigger_motor_->rpm(), info_.jam_current, info_.jam_speed);
        if (jammed && now >= jam_release_at_) {
            jam_release_at_ = now + info_.jam_release_time;
            log_warn("Trigger jam detected; pausing feed");
        }

        const bool can_feed = detail::trigger_feed_allowed(
            firing_, friction_enabled_, friction_is_ready, motors_online, now < jam_release_at_);
        co_await trigger_motor_->set(can_feed ? info_.trigger_speed : 0.0f);
        
        co_await roboctrl::wait_for(info_.control_time);
    } 
}

#endif // ROBOCTRL_TESTING
