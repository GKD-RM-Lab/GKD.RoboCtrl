/**
 * @file pid.h
 * @brief 通用 PID 控制器实现。
 * @details 提供可配置的 PID 基类以及基于线性误差与角度误差的具体别名，支持与 `controlled_motor` 组合使用。
 */
#pragma once
#include <cmath>
#include <algorithm>

#include "device/motor/base.hpp"
#include "controller.hpp"
#include "utils/controller.hpp"

namespace roboctrl::utils{
/**
 * @brief PID 控制器基础模板。
 * @tparam T 计算类型
 * @tparam error_measurer 误差测度函数
 */
template<typename T, auto error_measurer>
struct pid_base {
    T kp = 0;
    T ki = 0;
    T kd = 0;

    T max_out{};
    T max_iout{};

    using input_type = T;
    using state_type = T;

    /**
     * @brief 统一的参数封装，方便序列化。
     */
    struct params_type{
        T kp = 0;
        T ki = 0;
        T kd = 0;

        T max_out{};
        T max_iout{};
    };

    pid_base() = default;

    pid_base(const params_type& params)
        : kp(params.kp), ki(params.ki), kd(params.kd),
          max_out(params.max_out), max_iout(params.max_iout) {}

    /**
     * @brief 设置期望目标。
     */
    void set_target(T target) { target_ = target; }

    inline T target() const { return target_; }

    /**
     * @brief 根据当前值更新 PID 输出。
     * @details
     * 与传统 RM 风格 PID（error[0]/error[1] 差分）保持兼容：
     * - 误差缓存顺序：先 old 再 new
     * - 积分项：I += ki * error
     * - 微分项：D = kd * (error_curr - error_prev)
     * - 限幅顺序：先 Iout，再输出
     */
    void update(T current) {
        T error_prev = last_error_;
        T error_curr = error_measurer(current, target_);
        last_error_ = error_curr;

        T pout = kp * error_curr;

        integral_ += ki * error_curr;
        integral_ = std::clamp(integral_, -max_iout, max_iout);

        T derivative = kd * (error_curr - error_prev);

        T out = pout + integral_ + derivative;
        out = std::clamp(out, -max_out, max_out);

        output_ = out;
    }

    /**
     * @brief 清空积分项、误差缓存与输出。
     */
    void clean() {
        integral_ = 0;
        last_error_ = 0;
        output_ = 0;
        target_ = 0;
    }

    /**
     * @brief 获取最新的控制输出。
     */
    T state() const { return output_; }

private:
    T target_ = 0;       ///< 目标值
    T integral_ = 0;     ///< 积分项累积
    T last_error_ = 0;   ///< 上一次误差（等价 B 版 error[1]）
    T output_ = 0;       ///< 输出结果
};

namespace details{

/** @brief 线性误差计算方式。 */
constexpr auto linear_error = [](fp32 cur, fp32 target) {
    return target - cur;
};

/** @brief 角度误差计算方式，自动处理 2π 周期。 */
constexpr auto rad_error = [](fp32 cur, fp32 target) {
    fp32 diff = target - cur;
    diff = fmod(diff + M_PI, 2*M_PI);
    if (diff < 0) diff += 2*M_PI;
    diff -= M_PI;
    return diff;
};

}

/** @brief 线性 PID 控制器。 */
using linear_pid = pid_base<fp32, details::linear_error>;
/** @brief 角度 PID 控制器。 */
using rad_pid    = pid_base<fp32, details::rad_error>;

static_assert(utils::controller<linear_pid>);
static_assert(utils::controller<rad_pid>);

/** @brief 将线性 PID 附着于特定电机类型的别名。 */
template<device::motor motor_type>
using linear_pid_motor = device::controlled_motor<motor_type, linear_pid>;

/** @brief 将角度 PID 附着于特定电机类型的别名。 */
template<device::motor motor_type>
using rad_pid_motor = device::controlled_motor<motor_type, rad_pid>;

}
