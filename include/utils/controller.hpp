#pragma once

#include <utility>
#include <functional>
namespace roboctrl::utils{

template<typename T>
concept controller = requires (T c) {
    typename T::state_type;
    typename T::input_type;
    { c.update(std::declval<typename T::input_type>()) } -> std::same_as<void>;
    { c.state() } -> std::same_as<typename T::state_type>;
};

template<controller... Cs>
struct control_chain {
    template<std::size_t N>
    using state_type_n = typename std::tuple_element<N, std::tuple<typename Cs::state_type...>>::type;

    template<std::size_t N>
    using input_type_n = typename std::tuple_element<N, std::tuple<typename Cs::input_type...>>::type;

    using state_type = state_type_n<sizeof...(Cs) - 1>;
    using input_type = input_type_n<0>;

    std::tuple<Cs...> controllers_;

    control_chain(Cs&&... cs)
        : controllers_(std::forward<Cs>(cs)...) {}

    template<std::size_t N>
    void update(const input_type_n<N>& input) {
        std::get<N>(controllers_).update(input);

        if constexpr (N + 1 < sizeof...(Cs))
            update<N + 1>(std::get<N>(controllers_).state());
    }

    void update(const input_type& input) {
        update<0>(input);
    }

    const state_type& state() const {
        return std::get<sizeof...(Cs) - 1>(controllers_).state();
    }
};

struct pid{
    float kp = 0;
    float ki = 0;
    float kd = 0;
};

}