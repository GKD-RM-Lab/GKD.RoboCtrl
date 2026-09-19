#include "ctrl/motion_control.h"

#include "core/multiton.hpp"
#include "ctrl/robot.h"
#include "ctrl/shoot.h"
#include "ctrl/power_manager.h"
#include "device/aim_link.hpp"
#include "device/referee/referee.h"
#include "device/super_cap.h"
#include "device/remote_logger.hpp"

namespace roboctrl::ctrl {

bool motion_control::init(const info_type& info) {
    info_ = info;
    auto& robot = roboctrl::get<ctrl::robot>();
    chassis_ = robot.chassis();
    gimbal_ = robot.gimbal();
    secondary_gimbal_ = robot.secondary_gimbal();
    large_yaw_ = robot.large_yaw();
    if (!info.primary_aim_key.empty()) primary_aim_ = &roboctrl::get<device::aim_link>(info.primary_aim_key);
    if (!info.secondary_aim_key.empty()) secondary_aim_ = &roboctrl::get<device::aim_link>(info.secondary_aim_key);
    if (!info.navigation_key.empty()) navigation_ = &roboctrl::get<device::navigation_link>(info.navigation_key);
    if (!info.remote_logger_key.empty()) remote_logger_ = &roboctrl::get<device::remote_logger>(info.remote_logger_key);
    follow_.configure(info.follow_pid, info.follow_direction, info.spin_recenter_speed, info.follow_tolerance);
    auto& pad = roboctrl::get<device::control_pad>(info.control_pad_key);
    pad.on_update([this](const device::control_pad_state& input) {
        input_ = input;
        input_pending_ = true;
    });
    roboctrl::spawn(task());
    return true;
}

void motion_control::stop_outputs() {
    if (chassis_) {
        chassis_->set_planar_velocity({0.0f, 0.0f});
        chassis_->set_rotate_speed(0.0f);
    }
    auto& robot = roboctrl::get<ctrl::robot>();
    for (auto* gimbal : {gimbal_, secondary_gimbal_, large_yaw_})
        if (gimbal) gimbal->hold();
    if (info_.enable_shoot) {
        auto& shoot = roboctrl::get<ctrl::shoot>();
        shoot.set_firing(false);
        shoot.set_fire_permitted(false);
        shoot.set_friction_enabled(false);
    }
    if (auto* shoot = robot.secondary_shoot()) {
        shoot->set_firing(false);
        shoot->set_fire_permitted(false);
        shoot->set_friction_enabled(false);
    }
    follow_.reset();
    search_elapsed_ = 0;
    auto_aim_last_ = false;
    if (robot.state() == robot_state::NoForce) mapper_.reset();
    if (roboctrl::get<device::referee>().configured())
        roboctrl::get<device::referee>().set_ui_status({});
}

void motion_control::dispatch(const control_command& command, fp32 dt) {
    auto& robot = roboctrl::get<ctrl::robot>();
    if (auto_aim_last_ != command.auto_aim) {
        for (auto* gimbal : {gimbal_, secondary_gimbal_, large_yaw_})
            if (gimbal) gimbal->hold();
        auto_aim_last_ = command.auto_aim;
    }
    const auto primary_target = primary_aim_ ? primary_aim_->target() : std::nullopt;
    const auto secondary_target = secondary_aim_ ? secondary_aim_->target() : std::nullopt;
    const bool searching = command.auto_aim && info_.search_when_vision_stale &&
        !primary_target && (!secondary_gimbal_ || !secondary_target);
    if (searching) {
        if (robot.state() != robot_state::Search) {
            search_elapsed_ = 0;
            robot.set_state(robot_state::Search);
        }
        search_elapsed_ += dt;
    } else if (robot.state() == robot_state::Search) robot.set_state(robot_state::FollowGimbal);

    const auto aim = [&](device::gimbal_base* gimbal, const auto& target) {
        if (!gimbal) return false;
        if (command.auto_aim) {
            if (target) {
                gimbal->set_target_yaw(target->yaw);
                gimbal->set_target_pitch(target->pitch);
                return target->fire && gimbal->initialized() && gimbal->online();
            }
            if (searching) {
                gimbal->add_yaw(info_.search_yaw_speed * dt);
                gimbal->set_target_pitch(info_.search_pitch_center + info_.search_pitch_amplitude *
                    std::sin(search_elapsed_ * info_.search_pitch_speed));
            } else gimbal->hold();
            return false;
        }
        if (command.use_pitch_target) gimbal->set_target_pitch(command.pitch_target);
        else gimbal->add_pitch(command.pitch_delta);
        gimbal->add_yaw(command.yaw_delta);
        return gimbal->initialized() && gimbal->online();
    };
    const bool primary_permission = gimbal_ ? aim(gimbal_, primary_target) : !command.auto_aim;
    const bool secondary_permission = aim(secondary_gimbal_, secondary_target);

    // Keep the small head near its mechanical forward position without assuming
    // that independently booted IMUs share the same absolute yaw origin.
    if (large_yaw_ && gimbal_ && gimbal_->relative_yaw_valid())
        large_yaw_->set_target_yaw(large_yaw_->yaw() +
            info_.large_yaw_follow_direction * gimbal_->relative_yaw());

    vectorf velocity = command.velocity;
    if (info_.enable_navigation && command.auto_aim && navigation_) {
        const auto navigation = navigation_->command();
        velocity = navigation ? vectorf{.x = navigation->vx, .y = navigation->vy} : vectorf{};
    }
    if (chassis_) {
        auto* heading = large_yaw_ ? large_yaw_ : gimbal_;
        if (heading && !heading->relative_yaw_valid()) {
            // World-yaw hold may be enabled without a mechanical zero; a chassis
            // frame transform is still invalid and therefore cannot drive wheels.
            chassis_->set_planar_velocity({});
            chassis_->set_rotate_speed(0);
            follow_.reset();
        } else {
            const auto relative_yaw = heading ? heading->relative_yaw() : 0.0f;
            chassis_->set_planar_velocity(gimbal_to_chassis(velocity, relative_yaw));
            const bool follow = heading && info_.enable_follow && robot.state() != robot_state::NotFollow;
            chassis_->set_rotate_speed(follow ? follow_.update(command.rotate_speed, relative_yaw, dt)
                : command.rotate_speed);
        }
    }
    const bool permitted = primary_permission;
    if (info_.enable_shoot) {
        auto& shoot = roboctrl::get<ctrl::shoot>();
        shoot.set_friction_enabled(command.friction_enabled);
        shoot.set_fire_permitted(permitted);
        shoot.set_firing(command.auto_aim ? permitted : command.firing);
    }
    if (auto* shoot = robot.secondary_shoot()) {
        shoot->set_friction_enabled(command.friction_enabled);
        shoot->set_fire_permitted(secondary_permission);
        shoot->set_firing(command.auto_aim ? secondary_permission : command.firing);
    }
    auto& referee = roboctrl::get<device::referee>();
    auto& capacitor = roboctrl::get<device::super_cap>();
    if (referee.configured()) referee.set_ui_status({
        .friction_ready = info_.enable_shoot && roboctrl::get<ctrl::shoot>().friction_ready(),
        .auto_aim = command.auto_aim, .spinning = command.rotate_speed != 0.0f,
        .fire_permitted = (info_.enable_shoot && roboctrl::get<ctrl::shoot>().fire_allowed()) ||
            (robot.secondary_shoot() && robot.secondary_shoot()->fire_allowed()),
        .capacitor_percent = capacitor.configured() && !capacitor.offline() ?
            float(capacitor.energy()) / 255.0f * 100.0f : 0.0f});
}

awaitable<void> motion_control::send_telemetry() {
    const auto now = std::chrono::steady_clock::now();
    auto& referee = roboctrl::get<device::referee>();
    const auto& data = referee.data();
    const bool identity_fresh = referee.configured() && data.robot.fresh(now, referee.timeout());
    const bool red = identity_fresh ? data.robot.value.robot_id < 100 : info_.red_team;
    if (primary_aim_ && gimbal_ && gimbal_->online())
        co_await primary_aim_->send_posture(gimbal_->yaw(), gimbal_->pitch(), red);
    if (secondary_aim_ && secondary_gimbal_ && secondary_gimbal_->online())
        co_await secondary_aim_->send_posture(secondary_gimbal_->yaw(), secondary_gimbal_->pitch(), red);
    if (navigation_) {
        const auto* heading = large_yaw_ ? large_yaw_ : gimbal_;
        const float hp = identity_fresh && data.robot.value.max_hp > 0 ?
            std::clamp(float(data.robot.value.hp) / data.robot.value.max_hp, 0.0f, 1.0f) : 0.0f;
        const bool started = referee.configured() && data.game.fresh(now, referee.timeout()) &&
            data.game.value.progress == 4;
        if (heading && heading->online()) co_await navigation_->send_status(heading->yaw(), hp, started);
    }
}

awaitable<void> motion_control::send_debug_telemetry() {
    if (!remote_logger_) co_return;
    if (gimbal_) {
        co_await remote_logger_->push_value("gimbal.yaw.target", gimbal_->target_yaw());
        co_await remote_logger_->push_value("gimbal.yaw.measured", gimbal_->yaw());
        co_await remote_logger_->push_value("gimbal.yaw.rate", gimbal_->yaw_rate());
        co_await remote_logger_->push_value("gimbal.pitch.target", gimbal_->target_pitch());
        co_await remote_logger_->push_value("gimbal.pitch.measured", gimbal_->pitch());
    }
    const auto& power = roboctrl::get<power_manager>().status();
    co_await remote_logger_->push_value("chassis.power.requested", power.requested_power);
    co_await remote_logger_->push_value("chassis.power.allocated", power.allocated_power);
    co_await remote_logger_->push_value("robot.state", static_cast<int>(roboctrl::get<robot>().state()));
    if (info_.enable_shoot) {
        const auto& shoot = roboctrl::get<ctrl::shoot>();
        co_await remote_logger_->push_value("shoot.firing", shoot.firing());
        co_await remote_logger_->push_value("shoot.friction_ready", shoot.friction_ready());
        co_await remote_logger_->push_value("shoot.fire_allowed", shoot.fire_allowed());
    }
}

awaitable<void> motion_control::task() {
    auto previous = std::chrono::steady_clock::now();
    auto telemetry_at = previous;
    auto debug_at = previous;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = now - previous;
        const fp32 dt = elapsed > std::chrono::steady_clock::duration::zero() && elapsed <= info_.control_time * 5
            ? std::chrono::duration<fp32>(elapsed).count() : 0.f;
        previous = now;
        auto& robot = roboctrl::get<ctrl::robot>();
        const bool remote_online = !roboctrl::get<device::control_pad>(info_.control_pad_key).offline();
        bool fresh_input = false;
        if (!remote_online) {
            robot.set_state(robot_state::NoForce);
            mapper_.reset();
            command_ = {};
            input_pending_ = false;
        } else if (input_pending_) {
            input_pending_ = false;
            fresh_input = true;
            command_ = mapper_.update(input_);
            if (command_.arm_requested && robot.state() == robot_state::NoForce)
                robot.set_state(gimbal_ ? robot_state::FinishInit : robot_state::FollowGimbal);
        }
        auto command = command_;
        // Mouse deltas are per received packet, unlike the held RC stick.
        if (mapper_.keyboard_mode() && !fresh_input) command.yaw_delta = command.pitch_delta = 0.f;
        if (robot.state() == robot_state::FinishInit && robot.gimbals_initialized())
            robot.set_state(robot_state::FollowGimbal);
        else if (robot.state() != robot_state::NoForce && robot.state() != robot_state::FinishInit &&
                 !robot.gimbals_online()) robot.set_state(robot_state::NoForce);
        const bool output_allowed = robot.state() != robot_state::NoForce &&
            robot.state() != robot_state::FinishInit && robot.state() != robot_state::Idle;
        if (output_allowed) dispatch(command, dt);
        else stop_outputs();
        if (chassis_) co_await roboctrl::get<power_manager>().update(*chassis_, output_allowed, dt);
        if (now >= telemetry_at) {
            co_await send_telemetry();
            telemetry_at = now + 20ms;
        }
        if (now >= debug_at) {
            co_await send_debug_telemetry();
            debug_at = now + info_.telemetry_period;
        }
        co_await roboctrl::wait_for(info_.control_time);
    }
}
} // namespace roboctrl::ctrl
