/**
 * @file chassis/base.hpp
 * @brief 底盘抽象接口与运行时注册表。
 * @details 控制层只依赖本文件定义的速度和使能接口，具体轮组、电机和运动学由实现类负责。
 */
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

/** @brief 底盘的统一控制接口。 */
class chassis_base {
public:
    /** @brief 底盘初始化参数。 */
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
    /** @brief 设置平面速度 `(x, y)`，单位由具体底盘约定为 m/s。 */
    virtual void set_planar_velocity(vectorf) = 0;
    /** @brief 单独设置平面 x 方向速度。 */
    virtual void set_planar_x(fp32 value) = 0;
    /** @brief 单独设置平面 y 方向速度。 */
    virtual void set_planar_y(fp32 value) = 0;
    /** @brief 设置平面速度，兼容旧调用点。 */
    virtual void set_velocity(vectorf velocity) { set_planar_velocity(velocity); }
    /** @brief 同时设置平移和旋转速度。 */
    virtual void set_velocity(fp32 x, fp32 y, fp32 z) {
        set_planar_velocity({x, y});
        set_rotate_speed(z);
    }
    /** @brief 获取当前平面速度目标。 */
    virtual vectorf velocity() const = 0;
    /** @brief 设置绕 z 轴的旋转速度。 */
    virtual void set_rotate_speed(fp32) = 0;
    /** @brief 获取当前绕 z 轴的旋转速度目标。 */
    virtual fp32 rotate_speed() const = 0;
    /** @brief 启用或禁用底盘输出。 */
    virtual void set_enabled(bool) = 0;
};

/** @brief 判断类型是否实现底盘抽象。 */
template<typename T>
concept chassis = std::derived_from<T, chassis_base>;

/** @brief 具体底盘实现的运行时工厂注册表。 */
class chassis_registry {
public:
    using factory = std::function<chassis_base*(const std::any&)>;
    /** @brief 注册一个类型名及其工厂；重复类型名返回 false。 */
    static bool register_type(std::string type, factory creator);
    /** @brief 按类型名创建并初始化底盘。 */
    static chassis_base* create(std::string_view type, const std::any& info);
    /** @brief 获取当前已初始化的底盘。 */
    static chassis_base* current();
    /** @brief 按类型名初始化当前底盘。 */
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
