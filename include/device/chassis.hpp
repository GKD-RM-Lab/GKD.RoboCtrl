#pragma once

#include <chrono>
#include <concepts>
#include <string>
#include <string_view>
#include <functional>
#include <any>

#include "core/async.hpp"
#include "core/logger.h"
#include "device/motor/base.hpp"
#include "utils/pid.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::device {

class chassis_base {
public:
    virtual ~chassis_base() = default;
    virtual void set_planar_velocity(vectorf) = 0;
    virtual void set_planar_x(fp32 value) = 0;
    virtual void set_planar_y(fp32 value) = 0;
    virtual void set_velocity(vectorf velocity) { set_planar_velocity(velocity); }
    virtual void set_velocity(fp32 x, fp32 y, fp32 z) {
        set_planar_velocity({x, y});
        set_rotate_speed(z);
    }
    virtual vectorf velocity() const = 0;
    virtual void set_rotate_speed(fp32) = 0;
    virtual fp32 rotate_speed() const = 0;
    virtual void set_enabled(bool) = 0;
};

template<typename T>
concept chassis = std::derived_from<T, chassis_base>;

class mecanum_chassis final : public chassis_base,
                              public utils::singleton_base<mecanum_chassis>,
                              public logable<mecanum_chassis> {
public:
    using base_type = chassis_base;
    using utils::singleton_base<mecanum_chassis>::singleton_base;
    std::string desc() const { return "mecanum chassis"; }
    awaitable<void> task();

    void set_planar_velocity(vectorf v) override { velocity_ = v; }
    void set_planar_x(fp32 value) override { velocity_.x = value; }
    void set_planar_y(fp32 value) override { velocity_.y = value; }
    void set_velocity(vectorf v) override { set_planar_velocity(v); }
    vectorf velocity() const override { return velocity_; }
    void set_rotate_speed(fp32 v) override { rotate_speed_ = v; }
    fp32 rotate_speed() const override { return rotate_speed_; }
    void set_enabled(bool enabled) override { enabled_ = enabled; }

    struct info_type {
        using owner_type = mecanum_chassis;
        std::string left_front_motor {"left_front_motor"};
        std::string right_front_motor {"right_front_motor"};
        std::string left_rear_motor {"left_rear_motor"};
        std::string right_rear_motor {"right_rear_motor"};
        std::chrono::steady_clock::duration control_time {1ms};
    };

    bool init(const info_type&);

private:
    vectorf velocity_{};
    fp32 rotate_speed_{};
    fp32 max_wheel_speed_{2.5f};
    std::chrono::steady_clock::duration control_time_{1ms};
    motor_base* left_front_motor_ {nullptr};
    motor_base* right_front_motor_ {nullptr};
    motor_base* left_rear_motor_ {nullptr};
    motor_base* right_rear_motor_ {nullptr};
    bool enabled_{false};
};

class chassis_registry {
public:
    using factory = std::function<chassis_base*(const std::any&)>;
    static bool register_type(std::string type, factory creator);
    static chassis_base* create(std::string_view type, const std::any& info);
    static chassis_base* current();
    static bool init(std::string_view type, const std::any& info);
    static bool init(std::string_view type, const mecanum_chassis::info_type& info);
};

#ifndef ROBOCTRL_DETAIL_CAT
#define ROBOCTRL_DETAIL_CAT_IMPL(a, b) a##b
#define ROBOCTRL_DETAIL_CAT(a, b) ROBOCTRL_DETAIL_CAT_IMPL(a, b)
#endif

/** Register a singleton-backed chassis implementation during static initialization. */
#define ROBOCTRL_REGISTER_CHASSIS(TYPE_NAME, CLASS_NAME) \
    namespace { [[maybe_unused]] const bool ROBOCTRL_DETAIL_CAT(roboctrl_chassis_registered_, __LINE__) = [] { \
        return ::roboctrl::device::chassis_registry::register_type( \
            (TYPE_NAME), [](const std::any& value) -> ::roboctrl::device::chassis_base* { \
                const auto& info = std::any_cast<const CLASS_NAME::info_type&>(value); \
                ::roboctrl::init(info); \
                return &::roboctrl::get<CLASS_NAME>(); \
            }); \
    }(); }

static_assert(utils::singleton<mecanum_chassis>);
static_assert(chassis<mecanum_chassis>);
}
