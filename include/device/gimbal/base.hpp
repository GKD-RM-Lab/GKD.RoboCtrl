#pragma once

#include <any>
#include <chrono>
#include <concepts>
#include <functional>
#include <string>
#include <string_view>

#include "utils/pid.h"
#include "utils/utils.hpp"

namespace roboctrl::device {
using namespace std::chrono_literals;

class gimbal_base {
public:
    struct info_type {
        using owner_type = gimbal_base;
        std::string imu_key {"imu"};
        std::string yaw_motor_key {"gimbal_yaw_motor"};
        std::string pitch_motor_key {"gimbal_pitch_motor"};
        utils::rad_pid::params_type yaw_angle_pid {}, pitch_angle_pid {};
        fp32 yaw_direction {1.0f}, pitch_direction {1.0f};
        fp32 pitch_min {-0.3f}, pitch_max {0.3f};
        std::chrono::steady_clock::duration control_time {1ms};
    };

    virtual ~gimbal_base() = default;
    virtual fp32 yaw() const = 0;
    virtual fp32 pitch() const = 0;
    virtual void set_target_yaw(fp32) = 0;
    virtual void set_target_pitch(fp32) = 0;
    virtual void add_yaw(fp32) = 0;
    virtual void add_pitch(fp32) = 0;
    virtual void set_enabled(bool) = 0;
};

template<typename T>
concept gimbal = std::derived_from<T, gimbal_base>;

class gimbal_registry {
public:
    using factory = std::function<gimbal_base*(const std::any&)>;
    static bool register_type(std::string type, factory creator);
    static gimbal_base* create(std::string_view type, const std::any& info);
    static gimbal_base* current();
    static bool init(std::string_view type, const std::any& info);
};

#ifndef ROBOCTRL_DETAIL_CAT
#define ROBOCTRL_DETAIL_CAT_IMPL(a, b) a##b
#define ROBOCTRL_DETAIL_CAT(a, b) ROBOCTRL_DETAIL_CAT_IMPL(a, b)
#endif

#define ROBOCTRL_REGISTER_GIMBAL(TYPE_NAME, CLASS_NAME) \
    namespace { [[maybe_unused]] const bool ROBOCTRL_DETAIL_CAT(roboctrl_gimbal_registered_, __LINE__) = [] { \
        return ::roboctrl::device::gimbal_registry::register_type( \
            (TYPE_NAME), [](const std::any& value) -> ::roboctrl::device::gimbal_base* { \
                const auto& info = std::any_cast<const CLASS_NAME::info_type&>(value); \
                if (!::roboctrl::get<CLASS_NAME>().init(info)) return nullptr; \
                return &::roboctrl::get<CLASS_NAME>(); \
            }); \
    }(); }

} // namespace roboctrl::device
