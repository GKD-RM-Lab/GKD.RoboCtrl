#pragma once
#include "core/task_context.hpp"
#include "device/base.hpp"
#include "utils/concepts.hpp"
#include "utils/controller.hpp"
#include <type_traits>

namespace roboctrl::device{
    struct motor_base : public device_base {
protected:
    float angle_ {};
    float speed_ {};
    float torque_ {};

public:
    inline auto angle() const { return angle_; }
    inline auto speed() const { return speed_; }
    inline auto torque() const { return torque_; }
};

template <typename T>
concept motor =
    std::derived_from<typename T::device_type, motor_base> and
    requires(T::dev_type t) {
        { t.set(std::declval<float>()) } -> std::same_as<void>;
        { t.enable() } -> std::same_as<awaitable<void>>;
        { t.task() } -> std::same_as<awaitable<void>>;
    };

template<motor motor_type,utils::controller controller_type>
requires (std::is_convertible_v<float, typename controller_type::input_type> &&
    std::is_convertible_v<typename controller_type::state_type, float >)
struct controlled_motor{
    void set(float state){
        controller.update(state);
        motor.set(controller.state());
    }

    motor_type motor;
    controller_type controller;
};

} // namespace dev

namespace motor {
enum class dir : int {
    forward = 1,
    reverse = -1,
};
}