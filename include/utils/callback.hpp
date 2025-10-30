#pragma once

#include <concepts>
#include <functional>
#include <type_traits>
#include <vector>
#include <utility>
#include "core/task_context.hpp"

namespace roboctrl{

template<typename T,typename... Args>
concept callback_fn = 
    std::is_convertible_v<T, std::function<void(Args...)>> || 
    std::is_convertible_v<T, std::function<awaitable<void>(Args...)>>;

template<typename ...Args>
class callback {
public:
    explicit callback(task_context& ctx) : ctx_{ctx} {}

    void operator()(const Args&... args) {
        for (auto& fn : fns_)
            ctx_.spawn(fn(args...));
    }
    void add(std::function<awaitable<void>()> fn) {
        fns_.push_back(std::move(fn));
    }

    void add(std::function<void(Args...)> fn){
        fns_.push_back(std::move([fn](Args&&... args) -> awaitable<void> {
            fn(std::forward<Args>(args)...);
            co_return;
        }));
    }

private:
    std::vector<std::function<awaitable<void>(Args...)>> fns_;
    task_context& ctx_;
};

}