#pragma once

#include <algorithm>
#include <chrono>
#include <concepts>
#include <string>
#include <string_view>
#include <functional>
#include <any>

#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "device/imu/base.hpp"
#include "utils/pid.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::device {
class gimbal_base {
public:
    virtual ~gimbal_base() = default;
    virtual fp32 yaw() const = 0;
    virtual fp32 pitch() const = 0;
    virtual void set_target_yaw(fp32) = 0;
    virtual void set_target_pitch(fp32) = 0;
    virtual void set_yaw(fp32 value) { set_target_yaw(value); }
    virtual void set_pitch(fp32 value) { set_target_pitch(value); }
    virtual void set_yaw_target(fp32 value) { set_target_yaw(value); }
    virtual void set_pitch_target(fp32 value) { set_target_pitch(value); }
    virtual void add_yaw(fp32) = 0;
    virtual void add_pitch(fp32) = 0;
    virtual void set_enabled(bool) = 0;
};

template<typename T>
concept gimbal = std::derived_from<T, gimbal_base>;

class imu_gimbal final : public gimbal_base,
                         public utils::singleton_base<imu_gimbal>,
                         public logable<imu_gimbal> {
public:
    std::string desc() const { return "IMU gimbal"; }
    awaitable<void> task();
    fp32 yaw() const override { return yaw_; }
    void set_target_yaw(fp32 v) override { yaw_ = utils::rad_format(v); }
    void add_yaw(fp32 v) override { yaw_ = utils::rad_format(yaw_ + v); }
    fp32 pitch() const override { return pitch_; }
    void set_target_pitch(fp32 v) override { pitch_ = std::clamp(v, pitch_min_, pitch_max_); }
    void add_pitch(fp32 v) override { set_target_pitch(pitch_ + v); }
    void set_enabled(bool v) override { enabled_ = v; }
    void set_yaw(fp32 v) override { set_target_yaw(v); }
    void set_pitch(fp32 v) override { set_target_pitch(v); }
    void set_yaw_target(fp32 v) override { set_target_yaw(v); }
    void set_pitch_target(fp32 v) override { set_target_pitch(v); }

    struct info_type {
        using owner_type = imu_gimbal;
        std::string imu_key {"imu"};
        std::string yaw_motor_key {"gimbal_yaw_motor"};
        std::string pitch_motor_key {"gimbal_pitch_motor"};
        utils::rad_pid::params_type yaw_angle_pid {}, pitch_angle_pid {};
        fp32 yaw_direction {1.0f}, pitch_direction {1.0f};
        fp32 pitch_min {-0.3f}, pitch_max {0.3f};
        std::chrono::steady_clock::duration control_time {1ms};
    };
    bool init(const info_type&);
private:
    fp32 yaw_{}, pitch_{}, pitch_min_{-0.3f}, pitch_max_{0.3f};
    fp32 yaw_direction_{1.0f}, pitch_direction_{1.0f}, yaw_zero_{};
    bool yaw_zero_initialized_{false}, targets_initialized_{false}, enabled_{false};
    std::string imu_key_;
    std::chrono::steady_clock::duration control_time_{1ms};
    utils::rad_pid yaw_angle_pid_{}, pitch_angle_pid_{};
    motor_base* yaw_motor_ {nullptr};
    motor_base* pitch_motor_ {nullptr};
};

class gimbal_registry {
public:
    using factory = std::function<gimbal_base*(const std::any&)>;
    static bool register_type(std::string type, factory creator);
    static gimbal_base* create(std::string_view type, const std::any& info);
    static gimbal_base* current();
    static bool init(std::string_view type, const std::any& info);
    static bool init(std::string_view type, const imu_gimbal::info_type& info);
};

#ifndef ROBOCTRL_DETAIL_CAT
#define ROBOCTRL_DETAIL_CAT_IMPL(a, b) a##b
#define ROBOCTRL_DETAIL_CAT(a, b) ROBOCTRL_DETAIL_CAT_IMPL(a, b)
#endif

/** Register a singleton-backed gimbal implementation during static initialization. */
#define ROBOCTRL_REGISTER_GIMBAL(TYPE_NAME, CLASS_NAME) \
    namespace { [[maybe_unused]] const bool ROBOCTRL_DETAIL_CAT(roboctrl_gimbal_registered_, __LINE__) = [] { \
        return ::roboctrl::device::gimbal_registry::register_type( \
            (TYPE_NAME), [](const std::any& value) -> ::roboctrl::device::gimbal_base* { \
                const auto& info = std::any_cast<const CLASS_NAME::info_type&>(value); \
                ::roboctrl::init(info); \
                return &::roboctrl::get<CLASS_NAME>(); \
            }); \
    }(); }

static_assert(utils::singleton<imu_gimbal>);
static_assert(gimbal<imu_gimbal>);
}
