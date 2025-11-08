#pragma once
#include <chrono>
#include "core/logger.h"
#include "utils/concepts.hpp"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"
#include "core/multiton.hpp"
#include "core/task_context.hpp"
#include <concepts>
#include <type_traits>
#include <vector>

using namespace std::chrono_literals;

namespace roboctrl::device{

struct device_base :
    public utils::immovable_base, 
    public utils::not_copyable_base {
protected:
    const std::chrono::nanoseconds offline_timeout_;
    std::chrono::nanoseconds tick_time_ { 
        utils::now() - (offline_timeout_ == std::chrono::nanoseconds::max() ? 0ns : offline_timeout_)
    };

public:
    device_base(const std::chrono::nanoseconds offline_timeout);

    bool offline() { return offline_timeout_ == 0ms?false:utils::now() - tick_time_ > offline_timeout_; }
    void tick() { tick_time_ = utils::now(); }

    awaitable<void> task(){co_return ;}
};

template<typename T>
concept device = std::is_base_of_v<device_base, T> && roboctrl::owner<T> && requires (T dev) {
    {dev.task()} -> std::same_as<awaitable<void>>;
};


bool is_offline(auto&... devs)
    requires (requires(decltype(devs) d) {
        { d.offline() } -> std::same_as<bool>;
    } and ...) {
    return (devs.offline() or ...);
}
}