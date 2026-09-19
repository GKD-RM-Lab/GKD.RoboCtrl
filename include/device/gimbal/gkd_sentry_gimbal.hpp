#pragma once

#include <algorithm>
#include <chrono>
#include <string>

#include "device/gimbal/base.hpp"
#include "device/gimbal/feedback.hpp"
#include "device/motor/base.hpp"
#include "device/imu/base.hpp"
#include "utils/pid.h"

namespace roboctrl::device {

/** One physical IMU gimbal; registry owns independent primary/secondary/yaw stages. */
class gkd_sentry_gimbal final : public gimbal_base, public logable<gkd_sentry_gimbal> {
public:
    gkd_sentry_gimbal() = default;
    using info_type = gimbal_base::info_type;
    bool init(const info_type&);
    /** Bind physical or simulated devices without opening IO. */
    bool init(const info_type&, imu_base&, motor_base&, motor_base* pitch);
    /** One control tick, also usable by deterministic software simulation. */
    awaitable<void> update(fp32 dt);
    void start() override;
    std::string desc() const { return "GKD IMU gimbal " + info_.yaw_motor_key; }
    awaitable<void> task();
    fp32 yaw() const override { return measured_yaw_; }
    fp32 pitch() const override { return measured_pitch_; }
    fp32 yaw_rate() const override { return yaw_rate_; }
    fp32 target_yaw() const override { return yaw_target_; }
    fp32 target_pitch() const override { return pitch_target_; }
    fp32 relative_yaw() const override { return relative_yaw_; }
    bool relative_yaw_valid() const override { return online_ && info_.yaw_zero_calibrated; }
    bool online() const override { return online_; }
    bool initialized() const override { return initialized_ && online_; }
    void set_target_yaw(fp32 v) override { if (std::isfinite(v)) yaw_target_ = utils::rad_format(v); }
    void add_yaw(fp32 v) override { set_target_yaw(yaw_target_ + v); }
    void set_target_pitch(fp32 v) override {
        if (std::isfinite(v)) pitch_target_ = std::clamp(v, info_.pitch_min, info_.pitch_max);
    }
    void add_pitch(fp32 v) override { set_target_pitch(pitch_target_ + v); }
    void set_enabled(bool v) override;
    void set_recentering(bool v) override;
    void hold() override;

private:
    void reset_controllers();
    info_type info_;
    fp32 yaw_target_{}, pitch_target_{}, measured_yaw_{}, measured_pitch_{};
    fp32 relative_yaw_{}, yaw_rate_{};
    bool targets_initialized_{false}, initialized_{false}, enabled_{false};
    bool recentering_{false}, online_{false}, started_{false};
    bool drive_fault_latched_{false};
    gimbal_settle settle_;
    utils::rad_pid yaw_angle_pid_{}, pitch_angle_pid_{}, yaw_relative_pid_{};
    utils::linear_pid yaw_rate_pid_{}, pitch_rate_pid_{};
    imu_base* imu_{nullptr};
    motor_base* yaw_motor_{nullptr};
    motor_base* pitch_motor_{nullptr};
};

static_assert(gimbal<gkd_sentry_gimbal>);

} // namespace roboctrl::device
