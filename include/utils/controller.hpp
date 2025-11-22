/**
 * @file controller.hpp
 * @brief 控制器概念与串联控制链。
 * @details 统一约束控制器需暴露的输入、状态、参数接口，并提供 `control_chain` 帮助快速串联多个控制器。
 */
#pragma once

#include <utility>

namespace roboctrl::utils{

/**
 * @brief 控制器概念，约束 update/state/构造能力。
 * @tparam T 控制器类型
 */
template<typename T>
concept controller = requires (T c) {
    typename T::state_type;
    typename T::input_type;
    typename T::params_type;
    { T(std::declval<typename T::params_type>()) };
    { c.update(std::declval<typename T::input_type>()) } -> std::same_as<void>;
    { c.state() } -> std::same_as<typename T::state_type>;
};

/**
 * @brief 控制链，将多个控制器串联组合。
 * @tparam Cs 控制器类型参数包
 */
template<controller... Cs>
struct control_chain {
    static_assert(sizeof...(Cs) > 0);

    template<std::size_t N>
    using state_type_n = typename std::tuple_element<N, std::tuple<typename Cs::state_type...>>::type;

    template<std::size_t N>
    using input_type_n = typename std::tuple_element<N, std::tuple<typename Cs::input_type...>>::type;

    using state_type = state_type_n<sizeof...(Cs) - 1>;
    using input_type = input_type_n<0>;
    using params_type = std::tuple<typename Cs::params_type...>;

    std::tuple<Cs...> controllers_;

    /**
     * @brief 直接以控制器实例初始化。
     */
    explicit control_chain(Cs&&... cs)
        : controllers_(std::forward<Cs>(cs)...) {}

    /**
     * @brief 使用各控制器的参数构造实例。
     */
    explicit control_chain(params_type&& params)
        : controllers_(std::apply(
            [](auto&&... ps){ return std::make_tuple(Cs(std::forward<decltype(ps)>(ps))...); },
            std::move(params)))
    {}


    /**
     * @brief 推动控制链一次，依次更新所有控制器。
     * @param input 链的输入
     */
    void update(const input_type& input) {
        auto current_input = input;
        std::apply([&](auto&... ctrls){
            ((ctrls.update(current_input),
              current_input = ctrls.state()), ...);
        }, controllers_);
    }

    /**
     * @brief 读取链路末尾控制器的状态。
     */
    const state_type& state() const {
        return std::get<sizeof...(Cs) - 1>(controllers_).state();
    }
};

template<controller controller_type>
controller_type make_controller(const typename controller_type::info_type& info){
    return {info};
}

}
