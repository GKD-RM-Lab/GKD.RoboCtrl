#pragma once
#include <cmath>
#include <concepts>
#include <chrono>

#include "utils/controller.hpp"

namespace roboctrl::utils{


/**
 * @brief 一阶斜坡控制器 (Ramp)
 * @details
 * `ramp` 控制器用于限制输出变化速率，使系统输出以设定的最大速度逐步逼近目标输入。
 * 该控制器常用于以下场景：
 * - 电机转速平滑控制（防止瞬间加速导致电流冲击）
 * - 舵机或云台角度渐变控制（防止机械震动）
 * - 电流、电压缓启动控制
 *
 * 控制逻辑：
 * \f[
 *   \Delta y = \text{sign}(x_{\text{target}} - y_{\text{out}}) 
 *               \times \min(|x_{\text{target}} - y_{\text{out}}|, a_{\max} \cdot \Delta t)
 * \f]
 *
 * 其中：
 * - \f$ x_{\text{target}} \f$：输入目标
 * - \f$ y_{\text{out}} \f$：当前输出
 * - \f$ a_{\max} \f$：最大速度变化率（单位/秒）
 * - \f$ \Delta t \f$：两次调用间隔时间（秒）
 *
 * @tparam T 数值类型（例如 float 或 double）
 *
 * @note 
 * 本类使用 `std::chrono::steady_clock` 计时，不受系统时间调整影响。
 */
template<std::floating_point T>
class ramp {
public:
    /**
     * @brief 参数结构体
     * @details 包含最大变化速率（单位/秒）。
     */
    struct params_type {
        T acc; ///< 最大速度变化率 (单位/秒)
    };

    using input_type = T;  ///< 输入类型：目标值
    using state_type = T;  ///< 状态类型：当前输出值

public:
    ramp() = default;

    /**
     * @brief 构造函数
     * @param[in] params 斜坡控制参数（最大变化速率）
     *
     * @note 初始化时会记录当前系统时间，用于后续计算时间差。
     */
    explicit ramp(const params_type& params)
        : acc_{params.acc}, last_update_{std::chrono::steady_clock::now()} {}

    /**
     * @brief 更新输出值
     * @details
     * 计算当前到目标值的差，并根据时间差 \f$\Delta t\f$ 和最大速率限制 \f$a_{\max}\f$，
     * 按照下式调整输出：
     * \f[
     *   y_{\text{out}}(t + \Delta t) = y_{\text{out}}(t)
     *      + \text{sign}(x_{\text{target}} - y_{\text{out}}(t)) 
     *        \times \min(|x_{\text{target}} - y_{\text{out}}(t)|, a_{\max} \cdot \Delta t)
     * \f]
     *
     * @param[in] target 输入目标值
     */
    inline void update(T target) noexcept {
        using namespace std::chrono;
        const auto now = steady_clock::now();
        const std::chrono::duration<T> dur = now - last_update_; ///< 两次更新间隔时间（秒）
        last_update_ = now;

        const T dt = dur.count();
        const T diff = target - out_;
        const T max_step = acc_ * dt;

        if (std::fabs(diff) <= max_step)
            out_ = target;
        else
            out_ += std::copysign(max_step, diff);
    }

    /**
     * @brief 修改最大变化速率
     * @param[in] new_acc 新的最大变化速率 (单位/秒)
     */
    inline void set_acc(T new_acc) noexcept { acc_ = new_acc; }

    /**
     * @brief 清零输出值
     * @note 输出会被设为 0。
     */
    inline void reset() noexcept { out_ = T{0}; }

    /**
     * @brief 设置输出初值
     * @param[in] value 初始输出值
     */
    inline void reset(T value) noexcept { out_ = value; }

    /**
     * @brief 获取当前输出状态
     * @return 当前输出值
     */
    inline state_type state() const noexcept { return out_; }

private:
    T out_ = T{0}; ///< 当前输出值
    T acc_ = T{0}; ///< 最大变化速率 (单位/秒)
    std::chrono::steady_clock::time_point last_update_{}; ///< 上次更新时间
};

/**
 * @brief `float` 精度的 Ramp 控制器别名。
 */
using ramp_f = ramp<float>;

/**
 * @brief `double` 精度的 Ramp 控制器别名。
 */
using ramp_d = ramp<double>;

/**
 * @brief 编译期验证 ramp 满足 controller 概念。
 */
static_assert(utils::controller<ramp_f>);
static_assert(utils::controller<ramp_d>);

} // namespace control
