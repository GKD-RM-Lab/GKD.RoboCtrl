/**
 * @file device/motor/ref.hpp
 * @brief 任意电机类型的非拥有型、类型擦除引用。
 */
#pragma once

#include <stdexcept>
#include <utility>

#include "device/motor/base.hpp"

namespace roboctrl::device {

/**
 * @brief 为所有满足 `motor` concept 的类型提供统一引用接口。
 *
 * `motor_ref` 不拥有电机。它只保存由 multiton 管理的稳定对象地址，以及
 * `set`/`enable` 的类型擦除调用入口。绑定一次具体类型后，控制代码无需再写
 * `set_motor<ConcreteMotor>(key, value)`。
 *
 * @code
 * auto left = motor_ref::from<dji_motor>("left_front_motor");
 * co_await left.set(1.5f);
 * @endcode
 */
class motor_ref {
public:
    motor_ref() = default;

    template<motor Motor>
    explicit motor_ref(Motor& motor) noexcept
        : motor_{&motor},
          set_{&set_impl<Motor>},
          enable_{&enable_impl<Motor>}
    {}

    /**
     * @brief 根据 multiton key 绑定具体电机；具体类型只在这里出现一次。
     */
    template<motor Motor>
    [[nodiscard]]
    static motor_ref from(const typename Motor::info_type::key_type& key) {
        return motor_ref{roboctrl::get<Motor>(key)};
    }

    /**
     * @brief 从电机 info 自动推导其 owner 类型并绑定已初始化实例。
     */
    template<multiton_info Info>
        requires motor<owner_type_t<Info>>
    explicit motor_ref(const Info& info)
        : motor_ref{roboctrl::get<owner_type_t<Info>>(info.key())}
    {}

    [[nodiscard]] explicit operator bool() const noexcept {
        return motor_ != nullptr;
    }

    awaitable<void> set(fp32 value) const {
        require_bound();
        co_await set_(motor_, value);
    }

    awaitable<void> enable() const {
        require_bound();
        co_await enable_(motor_);
    }

    [[nodiscard]] fp32 angle() const {
        require_bound();
        return motor_->angle();
    }

    [[nodiscard]] fp32 angle_speed() const {
        require_bound();
        return motor_->angle_speed();
    }

    [[nodiscard]] fp32 rpm() const {
        require_bound();
        return motor_->rpm();
    }

    [[nodiscard]] fp32 torque() const {
        require_bound();
        return motor_->torque();
    }

    [[nodiscard]] fp32 linear_speed() const {
        require_bound();
        return motor_->linear_speed();
    }

    [[nodiscard]] bool offline() const {
        require_bound();
        return motor_->offline();
    }

private:
    using set_fn = awaitable<void> (*)(motor_base*, fp32);
    using enable_fn = awaitable<void> (*)(motor_base*);

    template<motor Motor>
    static awaitable<void> set_impl(motor_base* motor, fp32 value) {
        co_await static_cast<Motor*>(motor)->set(value);
    }

    template<motor Motor>
    static awaitable<void> enable_impl(motor_base* motor) {
        co_await static_cast<Motor*>(motor)->enable();
    }

    void require_bound() const {
        if (!motor_) {
            throw std::logic_error("motor_ref is not bound to a motor");
        }
    }

    motor_base* motor_ {nullptr};
    set_fn set_ {nullptr};
    enable_fn enable_ {nullptr};
};

} // namespace roboctrl::device
