/**
 * @file gimbal/base.hpp
 * @brief 云台抽象接口与运行时注册表。
 */
#pragma once

#include <any>
#include <chrono>
#include <concepts>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "utils/pid.h"
#include "utils/utils.hpp"

namespace roboctrl::device {
using namespace std::chrono_literals;

/** @brief 云台的统一姿态目标接口。 */
class gimbal_base {
public:
    /** @brief 云台初始化参数，包括 IMU、电机和角度限制。 */
    struct info_type {
        using owner_type = gimbal_base;
        std::string imu_key {"imu"};
        std::string yaw_motor_key {"gimbal_yaw_motor"};
        std::string pitch_motor_key {"gimbal_pitch_motor"};
        std::string yaw_motor_type {"dji"}, pitch_motor_type {"dji"};
        utils::rad_pid::params_type yaw_angle_pid {}, pitch_angle_pid {};
        utils::rad_pid::params_type yaw_relative_pid {};
        utils::linear_pid::params_type yaw_rate_pid {}, pitch_rate_pid {};
        fp32 yaw_zero {};
        fp32 yaw_angle_direction {1.0f}, yaw_recenter_direction {1.0f};
        bool yaw_zero_calibrated {false};
        bool yaw_only {false};
        bool yaw_current_control {true}, pitch_current_control {true};
        bool recenter_on_enable {true};
        fp32 init_tolerance {0.1f}, init_pitch {};
        std::chrono::steady_clock::duration init_settle_time {2000ms};
        fp32 yaw_direction {1.0f}, pitch_direction {1.0f};
        fp32 pitch_min {-0.3f}, pitch_max {0.3f};
        std::chrono::steady_clock::duration control_time {1ms};
    };

    virtual ~gimbal_base() = default;
    virtual void start() = 0;
    /** @brief 获取当前 yaw 姿态，单位 rad。 */
    virtual fp32 yaw() const = 0;
    /** @brief 获取当前 pitch 姿态，单位 rad。 */
    virtual fp32 pitch() const = 0;
    /** Measured mechanical yaw relative to the configured chassis-facing zero. */
    virtual fp32 relative_yaw() const = 0;
    virtual bool relative_yaw_valid() const = 0;
    virtual bool online() const = 0;
    virtual bool initialized() const = 0;
    /** Explicit enabled initialization state; may produce output only while enabled. */
    virtual void set_recentering(bool) = 0;
    virtual void hold() = 0;
    virtual fp32 yaw_rate() const = 0;
    virtual fp32 target_yaw() const = 0;
    virtual fp32 target_pitch() const = 0;
    /** @brief 设置 yaw 绝对目标，单位 rad。 */
    virtual void set_target_yaw(fp32) = 0;
    /** @brief 设置 pitch 绝对目标，单位 rad。 */
    virtual void set_target_pitch(fp32) = 0;
    /** @brief 增加 yaw 目标量，单位 rad。 */
    virtual void add_yaw(fp32) = 0;
    /** @brief 增加 pitch 目标量，单位 rad。 */
    virtual void add_pitch(fp32) = 0;
    /** @brief 启用或禁用云台输出。 */
    virtual void set_enabled(bool) = 0;
};

/** @brief 判断类型是否实现云台抽象。 */
template<typename T>
concept gimbal = std::derived_from<T, gimbal_base>;

/** @brief 具体云台实现的运行时工厂注册表。 */
class gimbal_registry {
public:
    using factory = std::function<std::unique_ptr<gimbal_base>(const std::any&)>;
    /** @brief 注册一个类型名及其工厂；重复类型名返回 false。 */
    static bool register_type(std::string type, factory creator);
    /** @brief 按类型名创建并初始化云台。 */
    static gimbal_base* create(std::string_view type, const std::any& info);
    /** @brief 获取当前已初始化的云台。 */
    static gimbal_base* current();
    /** @brief 按类型名初始化当前云台。 */
    static bool init(std::string_view type, const std::any& info);
};

#ifndef ROBOCTRL_DETAIL_CAT
#define ROBOCTRL_DETAIL_CAT_IMPL(a, b) a##b
#define ROBOCTRL_DETAIL_CAT(a, b) ROBOCTRL_DETAIL_CAT_IMPL(a, b)
#endif

#define ROBOCTRL_REGISTER_GIMBAL(TYPE_NAME, CLASS_NAME) \
    namespace { [[maybe_unused]] const bool ROBOCTRL_DETAIL_CAT(roboctrl_gimbal_registered_, __LINE__) = [] { \
        return ::roboctrl::device::gimbal_registry::register_type( \
            (TYPE_NAME), [](const std::any& value) -> std::unique_ptr<::roboctrl::device::gimbal_base> { \
                const auto& info = std::any_cast<const CLASS_NAME::info_type&>(value); \
                auto result = std::make_unique<CLASS_NAME>(); \
                if (!result->init(info)) return nullptr; \
                return result; \
            }); \
    }(); }

} // namespace roboctrl::device
