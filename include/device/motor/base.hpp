#pragma once
#include "core/task_context.hpp"
#include "device/base.hpp"
#include "utils/controller.hpp"
#include <cstddef>
#include <type_traits>
#include "io/base.hpp"
#include "core/multiton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::device{

struct motor_measure{
    uint16_t ecd = 0;
    int16_t speed_rpm = 0;
    int16_t given_current = 0;
    uint8_t temperate = 0;
};

struct motor_base : public device_base {
protected:
    float angle_ {}; //rad
    float angle_speed_ {}; //rad/s
    float torque_ {}; //A

    float radius_ {};

public:
    inline auto angle() const { return angle_; }
    inline auto angle_speed() const { return angle_speed_; }
    inline auto torque() const { return torque_; }
    inline auto linear_speed() const { return angle_speed_ * radius_; }
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

template<motor motor_type,utils::controller controller_type>
requires (std::is_convertible_v<float, typename controller_type::input_type> &&
    std::is_convertible_v<typename controller_type::state_type, float >)
struct controlled_motor{
    using motor_key_type = typename motor_type::info_type::key_type;

    controller_type controller;
    motor_key_type key;

    controlled_motor(const motor_key_type& key,controller_type controller):
        key{key},controller{controller}{}
    
    void set(float state){
        controller.update(state);
        roboctrl::get<motor_type>(key).set(controller.state());
    }
};

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
