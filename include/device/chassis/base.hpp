#pragma once

#include <any>
#include <chrono>
#include <concepts>
#include <functional>
#include <string>
#include <string_view>

#include "utils/utils.hpp"

namespace roboctrl::device {
using namespace std::chrono_literals;

class chassis_base {
public:
    struct info_type {
        using owner_type = chassis_base;
        std::string left_front_motor {"left_front_motor"};
        std::string right_front_motor {"right_front_motor"};
        std::string left_rear_motor {"left_rear_motor"};
        std::string right_rear_motor {"right_rear_motor"};
        std::chrono::steady_clock::duration control_time {1ms};
        fp32 max_rotate_speed {1.0f};
    };

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

class chassis_registry {
public:
    using factory = std::function<chassis_base*(const std::any&)>;
    static bool register_type(std::string type, factory creator);
    static chassis_base* create(std::string_view type, const std::any& info);
    static chassis_base* current();
    static bool init(std::string_view type, const std::any& info);
};

#ifndef ROBOCTRL_DETAIL_CAT
#define ROBOCTRL_DETAIL_CAT_IMPL(a, b) a##b
#define ROBOCTRL_DETAIL_CAT(a, b) ROBOCTRL_DETAIL_CAT_IMPL(a, b)
#endif

#define ROBOCTRL_REGISTER_CHASSIS(TYPE_NAME, CLASS_NAME) \
    namespace { [[maybe_unused]] const bool ROBOCTRL_DETAIL_CAT(roboctrl_chassis_registered_, __LINE__) = [] { \
        return ::roboctrl::device::chassis_registry::register_type( \
            (TYPE_NAME), [](const std::any& value) -> ::roboctrl::device::chassis_base* { \
                const auto& info = std::any_cast<const CLASS_NAME::info_type&>(value); \
                if (!::roboctrl::get<CLASS_NAME>().init(info)) return nullptr; \
                return &::roboctrl::get<CLASS_NAME>(); \
            }); \
    }(); }

} // namespace roboctrl::device
