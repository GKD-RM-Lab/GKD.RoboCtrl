/**
 * @file device/motor/base.hpp
 * @author Junity
 * @brief 电机基础组件。
 * @details 提供电机基础类motor_base和被控制算法控制的电机controlled_motor。
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "core/async.hpp"
#include "core/logger.h"
#include "core/multiton.hpp"
#include "device/base.hpp"
#include "utils/controller.hpp"
#include "utils/utils.hpp"
#include "io/base.hpp"

namespace roboctrl::device{

///@cond INTERNAL
namespace details{

    //通用电机测量数据。
struct motor_measure{
    uint16_t ecd = 0;
    int16_t speed_rpm = 0;
    int16_t given_current = 0;
    uint8_t temperate = 0;
};

struct motor_upload_pkg {
    uint8_t angle_h     : 1;
    uint8_t angle_l     : 1;
    uint8_t speed_h     : 1;
    uint8_t speed_l     : 1;
    uint8_t current_h   : 1;
    uint8_t current_l   : 1;
    uint8_t temperature : 1;
    uint8_t unused      : 1;
} __attribute__((packed));

inline motor_measure parse_motor_upload_pkg(io::byte_span data)
{
    motor_upload_pkg pkg = utils::from_bytes<motor_upload_pkg>(data);
    return {
        .ecd = utils::make_u16(pkg.angle_h, pkg.angle_l),
        .speed_rpm = utils::make_i16(pkg.speed_h, pkg.speed_l),
        .given_current = utils::make_i16(pkg.current_h, pkg.current_l),
        .temperate = static_cast<uint8_t>(pkg.temperature)
    };
}

}
/// @endcond 

struct motor_base : public device_base {
protected:
    float angle_ {}; //rad
    float angle_speed_ {}; //rad/s
    float torque_ {}; // A
    float radius_ {}; // m

public:
    /**
     * @brief 获取电机角度（单位为rad）
     * 
     * @return 电机角度（单位为rad）
     */
    inline fp32 angle() const { return angle_; }
    
    /**
     * @brief 获取电机角速度（单位为rad/s）
     * 
     * @return fp32 电机角速度（单位为rad/s）
     */
    inline fp32 angle_speed() const { return angle_speed_; }

    /**
     * @brief 获取电机转速（单位为rpm）
     * 
     * @return fp32 电机转速（单位为rpm）
     */
    inline fp32 rpm() const { return angle_speed_ * 60.f / (2.f * Pi_f); }
    
    /**
     * @brief 获取电机扭矩（单位为A）
     * 
     * @return fp32 电机扭矩（单位为A）
     */
    inline fp32 torque() const { return torque_; }
    
    /**
     * @brief 获取电机线速度（单位为m/s）
     * 
     * @return fp32 电机线速度（单位为m/s）
     */
    inline fp32 linear_speed() const { return angle_speed_ * radius_; }
    
    /**构造函数
    * @param offline_timeout 电机离线超时时间
    * @param radius 电机驱动轮半径，单位米
    */
    inline explicit motor_base(const std::chrono::nanoseconds offline_timeout,fp32 radius) : device_base{offline_timeout},radius_{radius}{}
};

template <typename T>
concept motor =
    owner<T> and
    std::derived_from<T, motor_base> and
    requires(T t) {
        { t.set(std::declval<float>()) } -> std::same_as<awaitable<void>>;
        { t.enable() } -> std::same_as<awaitable<void>>;
        { t.task() } -> std::same_as<awaitable<void>>;
    };


/**
 * @brief 被控制算法（例如PID）控制的电机
 * 
 * @tparam motor_type 电机类型
 * @tparam controller_type 控制器类型
 * @details controlled_motor可以像普通电机一样被set,但set传入的值会先被传入到控制器中，然后通过控制器计算得到电机的输出值，并传入电机。
 * 这个类常用于电机的PID控制。
 *
 * 示例：
 * 
 * ```cpp
 * controlled_motor<my_motor, linear_pid<float>> cmotor{"motor1", {1.0f, 0.1f, 0.01f}};
 * co_await cmotor.set(10.0f);
 * ```
 */
template<motor motor_type,utils::controller controller_type>
requires (std::is_convertible_v<float, typename controller_type::input_type> &&
    std::is_convertible_v<typename controller_type::state_type, float >)
struct controlled_motor{
    using motor_key_type = typename motor_type::info_type::key_type;

    controller_type controller;
    motor_key_type name;

    struct params_type{
        motor_key_type key;
        typename controller_type::params_type controller_params;
    };

    controlled_motor() = default;

    controlled_motor(const motor_key_type& name,const typename controller_type::params_type& controller_params):
        name{name},controller{controller_params}{}
    
    /**
     * @brief 设置电机目标状态
     * 
     * @param state 目标状态
     */
    awaitable<void> set(float state){
        controller.update(state);
        co_await roboctrl::get<motor_type>(name).set(controller.state());
    }

    /**
     * @brief 获取电机对象
     * 
     * @return motor_type& 电机对象
     */
    inline motor_type& motor() const {return roboctrl::get<motor_type>(name); }

    /**
     * @brief 获取电机角度（单位为rad）
     * 
     * @return fp32 电机角度（单位为rad）
     */
    inline fp32 angle() const { return motor().angle(); }

    /**
     * @brief 获取电机角速度（单位为rad/s）
     * 
     * @return fp32 电机角速度（单位为rad/s）
     */
    inline fp32 angle_speed() const { return motor().angle_speed(); }

    /**
     * @brief 获取电机扭矩（单位为A）
     * 
     * @return fp32 电机扭矩（单位为A）
     */
    inline fp32 torque() const { return motor().torque(); }

    /**
     * @brief 获取电机线速度（单位为m/s）
     * 
     * @return fp32 电机线速度（单位为m/s）
     */
    inline fp32 linear_speed() const { return motor().linear_speed(); }
};

/**
 * @brief 设置电机目标状态
 * 
 * @tparam motor_type 电机类型
 * @param key 电机名称
 * @param value 电机目标状态
 * @details 示例：
 *
 * ```cpp
 * co_await dev::set_motor<my_motor>("motor1", 10.0f);
 * ```
 */
template<motor motor_type>
awaitable<void> set_motor(const typename motor_type::info_type::key_type& key ,fp32 value){
    co_await roboctrl::get<motor_type>(key).set(value);
}

} // namespace dev

namespace motor {
enum class dir : int {
    forward = 1,
    reverse = -1,
};

}
