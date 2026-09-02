#pragma once

#include <algorithm>
#include <chrono>
#include <string>

#include "device/chassis/base.hpp"
#include "device/motor/base.hpp"
#include "utils/singleton.hpp"

namespace roboctrl::device {

class gkd_sentry_chassis final : public chassis_base,
                                  public utils::singleton_base<gkd_sentry_chassis>,
                                  public logable<gkd_sentry_chassis> {
public:
    using base_type = chassis_base;
    using utils::singleton_base<gkd_sentry_chassis>::singleton_base;

    std::string desc() const { return "GKD Sentry chassis"; }
    awaitable<void> task();

    void set_planar_velocity(vectorf v) override { velocity_ = v; }
    void set_planar_x(fp32 value) override { velocity_.x = value; }
    void set_planar_y(fp32 value) override { velocity_.y = value; }
    vectorf velocity() const override { return velocity_; }
    void set_rotate_speed(fp32 v) override {
        rotate_speed_ = std::clamp(v, -max_rotate_speed_, max_rotate_speed_);
    }
    fp32 rotate_speed() const override { return rotate_speed_; }
    void set_enabled(bool enabled) override { enabled_ = enabled; }

    using info_type = chassis_base::info_type;

    bool init(const info_type&);

private:
    awaitable<void> speed_decomposition();

    vectorf velocity_{};
    fp32 rotate_speed_{};
    fp32 max_wheel_speed_{2.5f};
    fp32 max_rotate_speed_{1.0f};
    std::chrono::steady_clock::duration control_time_{1ms};
    motor_base* left_front_motor_{nullptr};
    motor_base* right_front_motor_{nullptr};
    motor_base* left_rear_motor_{nullptr};
    motor_base* right_rear_motor_{nullptr};
    bool enabled_{false};
};

static_assert(utils::singleton<gkd_sentry_chassis>);
static_assert(chassis<gkd_sentry_chassis>);

} // namespace roboctrl::device
