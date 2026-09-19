#include "ctrl/shoot.h"
#include "device/motor/dji.h"
#include "device/referee/referee.h"
#include <cmath>
#include <stdexcept>

using namespace roboctrl::ctrl;

namespace {

bool trigger_jammed(roboctrl::fp32 current, roboctrl::fp32 rpm,
    roboctrl::fp32 current_threshold, roboctrl::fp32 speed_threshold)
{
    return std::isfinite(current) && std::isfinite(rpm) &&
        std::fabs(current) > current_threshold && std::fabs(rpm) < speed_threshold;
}

bool trigger_feed_allowed(bool firing, bool friction_enabled, bool friction_ready,
    bool motors_online, bool jam_hold_active)
{
    return firing && friction_enabled && friction_ready && motors_online && !jam_hold_active;
}

bool referee_allows_feed(unsigned caliber)
{
    auto& referee = roboctrl::get<roboctrl::device::referee>();
    if (!referee.configured() || referee.offline()) return false;
    const auto& data = referee.data();
    const auto now = std::chrono::steady_clock::now();
    const auto timeout = referee.timeout();
    if (!data.game.fresh(now, timeout) || !data.robot.fresh(now, timeout) ||
        !data.ammunition.fresh(now, timeout)) return false;
    return data.game.value.progress == 4 && data.robot.value.hp != 0 &&
        data.robot.value.shooter_power &&
        (caliber == 42 ? data.ammunition.value.bullets_42 : data.ammunition.value.bullets_17) > 0;
}

} // namespace

bool shoot::init(const shoot::info_type& info)
{
    return init(info, roboctrl::get<device::dji_motor>(info.left_friction_motor),
        roboctrl::get<device::dji_motor>(info.right_friction_motor),
        roboctrl::get<device::dji_motor>(info.trigger_motor));
}

bool shoot::init(const info_type& info, device::motor_base& left,
    device::motor_base& right, device::motor_base& trigger)
{
    if (initialized_) throw std::logic_error("shoot already initialized");
    if (&left == &right || &left == &trigger || &right == &trigger ||
        !std::isfinite(info.friction_params.acc) || info.friction_params.acc < 0 ||
        !std::isfinite(info.friction_max_speed) || info.friction_max_speed <= 0 ||
        !std::isfinite(info.trigger_speed) || !std::isfinite(info.friction_ready_speed) ||
        info.friction_ready_speed <= 0 || info.friction_ready_speed > info.friction_max_speed ||
        !std::isfinite(info.jam_current) || info.jam_current < 0 ||
        !std::isfinite(info.jam_speed) || info.jam_speed < 0 ||
        info.control_time <= std::chrono::steady_clock::duration::zero() ||
        info.jam_release_time < std::chrono::steady_clock::duration::zero() ||
        (info.bullet_caliber != 17 && info.bullet_caliber != 42))
        throw std::invalid_argument("invalid shoot parameters/bindings");
    info_ = info;
    friction_ramp_ = utils::ramp_f{info_.friction_params};
    left_friction_motor_ = &left;
    right_friction_motor_ = &right;
    trigger_motor_ = &trigger;
    initialized_ = true;
    set_enabled(false);
    log_info("Shoot initiated");
    return true;
}

void shoot::start()
{
    if (started_) return;
    if (!initialized_) throw std::logic_error("shoot start before init");
    started_ = true;
    roboctrl::spawn(task());
}

void shoot::set_enabled(bool enabled)
{
    enabled = enabled && !roboctrl::async::shutdown_requested();
    enabled_ = enabled && initialized_;
    if (!enabled_) {
        firing_ = false;
        friction_enabled_ = false;
        fire_permitted_ = false;
        friction_ramp_.reset();
        jam_release_at_ = {};
    }
    for (auto* motor : {left_friction_motor_, right_friction_motor_, trigger_motor_})
        if (motor) motor->set_enabled(enabled_);
}

void shoot::set_firing(bool state)
{
    firing_ = enabled_ && state;
}

void shoot::set_friction_enabled(bool state)
{
    friction_enabled_ = enabled_ && state;
    if (!friction_enabled_) firing_ = false;
}

bool shoot::friction_ready() const
{
    if (!initialized_) return false;
    const auto left = left_friction_motor_->linear_speed();
    const auto right = right_friction_motor_->linear_speed();
    return std::isfinite(left) && std::isfinite(right) &&
        std::fabs(left) >= info_.friction_ready_speed &&
        std::fabs(right) >= info_.friction_ready_speed;
}

bool shoot::fire_allowed() const
{
    if (!enabled_ || !fire_permitted_ || !initialized_ ||
        !std::isfinite(trigger_motor_->angle_speed()) || !std::isfinite(trigger_motor_->current_feedback_raw())) return false;
    const bool motors_online = !left_friction_motor_->offline() &&
        !right_friction_motor_->offline() && !trigger_motor_->offline();
    return trigger_feed_allowed(firing_, friction_enabled_, friction_ready(), motors_online,
        std::chrono::steady_clock::now() < jam_release_at_) &&
        (!info_.enforce_referee || referee_allows_feed(info_.bullet_caliber));
}

roboctrl::awaitable<void> shoot::update(fp32 dt)
{
    if (!initialized_) co_return;
    const bool motors_online = !left_friction_motor_->offline() &&
        !right_friction_motor_->offline() && !trigger_motor_->offline();
    if (!enabled_ || !motors_online) {
        friction_ramp_.reset();
        co_await left_friction_motor_->set(0);
        co_await right_friction_motor_->set(0);
        co_await trigger_motor_->set_angle_speed(0);
        jam_release_at_ = {};
        co_return;
    }

    if (!std::isfinite(dt) || dt < 0 || dt > std::chrono::duration<fp32>(info_.control_time * 5).count()) dt = 0;
    friction_ramp_.update(friction_enabled_ ? info_.friction_max_speed : 0.0f, dt);
    co_await left_friction_motor_->set(-friction_ramp_.state());
    co_await right_friction_motor_->set(friction_ramp_.state());

    const auto now = std::chrono::steady_clock::now();
    if (firing_ && friction_enabled_ && friction_ready() &&
        trigger_jammed(trigger_motor_->current_feedback_raw(), trigger_motor_->rpm(), info_.jam_current, info_.jam_speed) &&
        now >= jam_release_at_) {
        jam_release_at_ = now + info_.jam_release_time;
        log_warn("Trigger jam detected; pausing feed");
    }
    co_await trigger_motor_->set_angle_speed(fire_allowed() ? info_.trigger_speed : 0.0f);
}

roboctrl::awaitable<void> shoot::task()
{
    auto previous = std::chrono::steady_clock::time_point{};
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto dt = previous == std::chrono::steady_clock::time_point{} ? 0.f :
            std::chrono::duration<fp32>(now - previous).count();
        previous = now;
        co_await update(dt);
        co_await roboctrl::wait_for(info_.control_time);
    }
}
