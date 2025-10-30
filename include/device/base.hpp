#pragma once
#include <chrono>
#include "utils/concepts.hpp"
#include "utils/utils.hpp"

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
    device_base(const std::chrono::nanoseconds offline_timeout) : offline_timeout_ { offline_timeout } {}

    bool offline() { return utils::now() - tick_time_ > offline_timeout_; }
    void tick() { tick_time_ = utils::now(); }
};

bool is_offline(auto&... devs)
    requires (requires(decltype(devs) d) {
        { d.offline() } -> std::same_as<bool>;
    } and ...) {
    return (devs.offline() or ...);
}
}