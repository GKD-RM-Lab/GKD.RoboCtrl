#pragma once

#include <concepts>
#include <functional>
#include <type_traits>
#include <vector>
#include <utility>
#include "core/task_context.hpp"

namespace roboctrl {

template<typename F, typename... Args>
concept callback_fn = std::invocable<F, Args...> &&
    (std::same_as<std::invoke_result_t<F, Args...>, void> ||
     std::same_as<std::invoke_result_t<F, Args...>, awaitable<void>>);

template<typename... Args>
class callback {
public:
    callback() = default;

    template<typename... CallArgs>
    void operator()(task_context& ctx, CallArgs&&... args) const {
        for (auto const& fn : fns_) {
            ctx.spawn(fn(std::forward<CallArgs>(args)...));
        }
    }

    template<callback_fn<Args...> F>
    void add(F&& f) {
        using result_t = std::invoke_result_t<F, Args...>;
        if constexpr (std::same_as<result_t, awaitable<void>>) {
            fns_.push_back(std::function<awaitable<void>(Args...)>(std::forward<F>(f)));
        } else {
            fns_.push_back(std::function<awaitable<void>(Args...)>(
                [fn = std::forward<F>(f)](Args... args) -> awaitable<void> {
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
