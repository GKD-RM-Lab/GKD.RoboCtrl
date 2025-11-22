/**
 * @file callback.hpp
 * @brief 协程回调收集器。
 * @details 允许注册一组同步或协程函数，在指定任务上下文中串行调度执行。
 */
#pragma once

#include <concepts>
#include <functional>
#include <type_traits>
#include <vector>
#include <utility>

#include "core/async.hpp"

namespace roboctrl {

/**
 * @brief 约束回调类型可被调用且返回 `void` 或 `awaitable<void>`。
 * @tparam F 可调用对象类型
 * @tparam Args 形参列表
 */

template<typename F, typename... Args>
concept callback_fn = std::invocable<F, Args...> &&
    (std::same_as<std::invoke_result_t<F, Args...>, void> ||
     std::same_as<std::invoke_result_t<F, Args...>, awaitable<void>>);

/**
 * @brief 管理一组协程回调的容器。
 * @tparam Args 回调参数类型
 */
template<typename... Args>
class callback {
public:
    callback() = default;

    /**
     * @brief 触发所有回调。
     * @param args 透传给回调的参数
     */
    template<typename... CallArgs>
    void operator()(CallArgs&&... args) const {
        for (auto const& fn : fns_) {
            roboctrl::spawn(fn(std::forward<CallArgs>(args)...));
        }
    }

    /**
     * @brief 添加一个新的回调。
     * @tparam F 满足 `callback_fn` 的函数或函子
     */
    template<callback_fn<Args...> F>
    void add(F&& f) {
        using result_t = std::invoke_result_t<F, Args...>;
        if constexpr (std::same_as<result_t, awaitable<void>>) {
            fns_.push_back(std::function<awaitable<void>(Args...)>(std::forward<F>(f)));
        } else {
            fns_.push_back(std::function<awaitable<void>(Args...)>(
                [fn = std::forward<F>(f)](Args... args) mutable -> awaitable<void> {
                    fn(args...);
                    co_return;
                }
            ));
        }
    }

private:
    std::vector<std::function<awaitable<void>(Args...)>> fns_;
};

} // namespace roboctrl
