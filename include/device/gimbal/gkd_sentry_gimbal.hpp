#pragma once

#include <algorithm>
#include <chrono>
#include <string>

#include "device/gimbal/base.hpp"
#include "device/motor/base.hpp"
#include "device/imu/base.hpp"
#include "utils/pid.h"
#include "utils/singleton.hpp"

namespace roboctrl::device {

class gkd_sentry_gimbal final : public gimbal_base,
                                public utils::singleton_base<gkd_sentry_gimbal>,
                                public logable<gkd_sentry_gimbal> {
public:
    using utils::singleton_base<gkd_sentry_gimbal>::singleton_base;

    std::string desc() const { return "GKD Sentry gimbal"; }
    awaitable<void> task();
    fp32 yaw() const override { return yaw_; }
    void set_target_yaw(fp32 v) override { yaw_ = utils::rad_format(v); }
    void add_yaw(fp32 v) override { yaw_ = utils::rad_format(yaw_ + v); }
    fp32 pitch() const override { return pitch_; }
    void set_target_pitch(fp32 v) override { pitch_ = std::clamp(v, pitch_min_, pitch_max_); }
    void add_pitch(fp32 v) override { set_target_pitch(pitch_ + v); }
    void set_enabled(bool v) override { enabled_ = v; }

    using info_type = gimbal_base::info_type;

    bool init(const info_type&);

private:
    fp32 yaw_{}, pitch_{}, pitch_min_{-0.3f}, pitch_max_{0.3f};
    fp32 yaw_direction_{1.0f}, pitch_direction_{1.0f}, yaw_zero_{};
    bool yaw_zero_initialized_{false}, targets_initialized_{false}, enabled_{false};
    std::string imu_key_;
    std::chrono::steady_clock::duration control_time_{1ms};
    utils::rad_pid yaw_angle_pid_{}, pitch_angle_pid_{};
    motor_base* yaw_motor_{nullptr};
    motor_base* pitch_motor_{nullptr};
};

static_assert(utils::singleton<gkd_sentry_gimbal>);
static_assert(gimbal<gkd_sentry_gimbal>);

} // namespace roboctrl::device
