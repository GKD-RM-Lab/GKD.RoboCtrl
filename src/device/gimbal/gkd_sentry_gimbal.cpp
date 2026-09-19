#include "device/gimbal/gkd_sentry_gimbal.hpp"
#include "core/async.hpp"
#include "device/imu/serial_imu.hpp"
#include "device/motor/lookup.hpp"

using namespace roboctrl;
using namespace roboctrl::device;

ROBOCTRL_REGISTER_GIMBAL("device.gkd_sentry_gimbal.v1", roboctrl::device::gkd_sentry_gimbal);
ROBOCTRL_REGISTER_GIMBAL("device.standard_imu_2axis_gimbal.v1", roboctrl::device::gkd_sentry_gimbal);
ROBOCTRL_REGISTER_GIMBAL("ctrl.standard_imu_2axis_gimbal.v1", roboctrl::device::gkd_sentry_gimbal);

void gkd_sentry_gimbal::reset_controllers() {
    yaw_angle_pid_.clean(); pitch_angle_pid_.clean(); yaw_relative_pid_.clean();
    yaw_rate_pid_.clean(); pitch_rate_pid_.clean();
}

void gkd_sentry_gimbal::hold() {
    yaw_target_ = measured_yaw_;
    pitch_target_ = std::clamp(measured_pitch_, info_.pitch_min, info_.pitch_max);
}

void gkd_sentry_gimbal::set_enabled(bool enabled) {
    enabled = enabled && !async::shutdown_requested();
    if (enabled_ != enabled) {
        reset_controllers();
        settle_.reset();
        initialized_ = false;
        targets_initialized_ = false;
    }
    enabled_ = enabled;
    if (!enabled) drive_fault_latched_ = false;
    // Native velocity control can hold torque at zero speed. Only the valid
    // update path may enable it after checking dependency/calibration gates.
    if (yaw_motor_ && (!enabled || info_.yaw_current_control)) yaw_motor_->set_enabled(enabled);
    if (pitch_motor_ && (!enabled || info_.pitch_current_control)) pitch_motor_->set_enabled(enabled);
}

void gkd_sentry_gimbal::set_recentering(bool enabled) {
    if (recentering_ != enabled) {
        reset_controllers();
        settle_.reset();
        if (enabled) initialized_ = false;
        else hold();
    }
    recentering_ = enabled;
}

roboctrl::awaitable<void> gkd_sentry_gimbal::update(fp32 dt) {
    if (!std::isfinite(dt) || dt <= 0 || dt > std::chrono::duration<fp32>(info_.control_time * 5).count()) {
        reset_controllers();
        settle_.reset();
        dt = 0;
    }
    if (yaw_motor_->faulted() || (pitch_motor_ && pitch_motor_->faulted()))
        drive_fault_latched_ = true;
    const auto angle = imu_->angle();
    const auto gyro = imu_->gyro();
    const auto motor_angle = yaw_motor_->angle();
    const fp32 raw_relative = motor_angle - info_.yaw_zero;
    const bool finite_feedback = std::isfinite(angle.yaw) && std::isfinite(angle.pitch) &&
        std::isfinite(angle.roll) && std::isfinite(raw_relative) &&
        std::isfinite(gyro.x) && std::isfinite(gyro.y) && std::isfinite(gyro.z);
    const fp32 projected_yaw_rate = finite_feedback ? compensated_yaw_rate(angle.pitch, gyro) : 0.0f;
    online_ = finite_feedback && std::isfinite(projected_yaw_rate) &&
        !drive_fault_latched_ && !imu_->offline() && !yaw_motor_->offline() &&
        (!pitch_motor_ || !pitch_motor_->offline());
    // Publish only a complete valid snapshot; hold/debug telemetry keep the last
    // finite pose while invalid feedback clears readiness and actuator outputs.
    if (online_) {
        measured_yaw_ = angle.yaw;
        measured_pitch_ = angle.pitch;
        relative_yaw_ = utils::rad_format(raw_relative);
        yaw_rate_ = projected_yaw_rate;
    }

    if (!online_) {
        targets_initialized_ = initialized_ = false;
        settle_.reset();
    }
    if (!targets_initialized_ || !enabled_) {
        hold();
        targets_initialized_ = online_;
    }
    const bool calibration_blocked = recentering_ && info_.recenter_on_enable &&
        !info_.yaw_zero_calibrated;
    if (!enabled_ || !online_ || calibration_blocked) {
        reset_controllers();
        // Use the same explicit mode as the active path, so a stale current
        // command cannot survive a switch to a zero speed target.
        if (info_.yaw_current_control) co_await yaw_motor_->set_current(0.0f);
        else {
            yaw_motor_->set_enabled(false);
            co_await yaw_motor_->set_angle_speed(0.0f);
        }
        if (pitch_motor_) {
            if (info_.pitch_current_control) co_await pitch_motor_->set_current(0.0f);
            else {
                pitch_motor_->set_enabled(false);
                co_await pitch_motor_->set_angle_speed(0.0f);
            }
        }
    } else {
        if (!info_.yaw_current_control) yaw_motor_->set_enabled(true);
        if (pitch_motor_ && !info_.pitch_current_control) pitch_motor_->set_enabled(true);
        fp32 yaw_speed;
        if (recentering_ && info_.recenter_on_enable) {
            yaw_relative_pid_.update(0.0f, relative_yaw_, dt);
            yaw_speed = info_.yaw_recenter_direction * yaw_relative_pid_.state();
            pitch_target_ = info_.init_pitch;
            initialized_ = settle_.update(enabled_, online_, relative_yaw_,
                pitch_motor_ ? utils::rad_format(info_.init_pitch - measured_pitch_) : 0.0f,
                info_.init_tolerance, std::chrono::duration<fp32>(info_.init_settle_time).count(), dt);
            yaw_target_ = measured_yaw_;
        } else {
            yaw_angle_pid_.update(yaw_target_, measured_yaw_, dt);
            yaw_speed = info_.yaw_angle_direction * yaw_angle_pid_.state();
            initialized_ = true;
        }
        // Motor output directions belong to the drive command, not the
        // physical feedback, whose sign is configured once at the IMU.
        if (info_.yaw_current_control) {
            yaw_rate_pid_.update(yaw_speed, yaw_rate_, dt);
            co_await yaw_motor_->set_current(info_.yaw_direction * yaw_rate_pid_.state());
        } else co_await yaw_motor_->set_angle_speed(info_.yaw_direction * yaw_speed);
        if (pitch_motor_) {
            pitch_angle_pid_.update(pitch_target_, measured_pitch_, dt);
            if (info_.pitch_current_control) {
                pitch_rate_pid_.update(pitch_angle_pid_.state(), gyro.y, dt);
                co_await pitch_motor_->set_current(info_.pitch_direction * pitch_rate_pid_.state());
            } else co_await pitch_motor_->set_angle_speed(info_.pitch_direction * pitch_angle_pid_.state());
        }
    }
}

roboctrl::awaitable<void> gkd_sentry_gimbal::task() {
    auto previous = std::chrono::steady_clock::now();
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const fp32 dt = std::chrono::duration<fp32>(now - previous).count();
        previous = now;
        co_await update(dt);
        co_await wait_for(info_.control_time);
    }
}

bool gkd_sentry_gimbal::init(const info_type& info) {
    auto& imu = roboctrl::get<serial_imu>(info.imu_key);
    auto& yaw = find_motor(info.yaw_motor_type, info.yaw_motor_key);
    auto* pitch = info.yaw_only ? nullptr : &find_motor(info.pitch_motor_type, info.pitch_motor_key);
    return init(info, imu, yaw, pitch);
}

bool gkd_sentry_gimbal::init(const info_type& info, imu_base& imu, motor_base& yaw, motor_base* pitch) {
    if (started_ || (!info.yaw_only && !pitch)) return false;
    info_ = info;
    imu_ = &imu;
    yaw_motor_ = &yaw;
    pitch_motor_ = info.yaw_only ? nullptr : pitch;
    if ((info.yaw_current_control && !yaw_motor_->supports_current_control()) ||
        (pitch_motor_ && info.pitch_current_control && !pitch_motor_->supports_current_control())) {
        log_error("Gimbal requested an unsupported current control mode");
        return false;
    }
    yaw_angle_pid_ = utils::rad_pid{info.yaw_angle_pid};
    pitch_angle_pid_ = utils::rad_pid{info.pitch_angle_pid};
    yaw_relative_pid_ = utils::rad_pid{info.yaw_relative_pid};
    yaw_rate_pid_ = utils::linear_pid{info.yaw_rate_pid};
    pitch_rate_pid_ = utils::linear_pid{info.pitch_rate_pid};
    return true;
}

void gkd_sentry_gimbal::start() {
    if (started_) return;
    started_ = true;
    roboctrl::spawn(task());
}
