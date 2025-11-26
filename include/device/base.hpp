/**
 * @file base.hpp
 * @brief 设备抽象基类与通用概念。
 * @details 定义所有设备需要遵循的生命周期接口，以及离线检测、任务执行等辅助功能。
 */
#pragma once
#include <chrono>
#include <concepts>
#include <type_traits>
#include <vector>

#include "core/logger.h"
#include "core/multiton.hpp"
#include "core/async.hpp"
#include "utils/concepts.hpp"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

using namespace std::chrono_literals;

/**
 * @brief 设备模块
 * 
 */
namespace roboctrl::device{

/**
 * @brief 设备基础类，提供判断设备离线的基础功能
 * @details 这个类主要提供判断设备是否离线的功能，设备离线的判断是基于心跳机制实现的。
 *
 * 设备类应当继承自这个类，并在适当的时候调用tick()方法来更新心跳时间。
 * 如果设备在指定的离线超时时间内没有收到心跳，则认为设备离线。
 * 
 * 此外，我们默认每个设备都有自己的task()，但有的设备可能没有，因此在这里提供一个空的task()实现。
 */
struct device_base :
    public utils::immovable_base, 
    public utils::not_copyable_base {
protected:
    const std::chrono::nanoseconds offline_timeout_; ///< 设备离线超时时间
    std::chrono::nanoseconds tick_time_ {  
        utils::now() - (offline_timeout_ == std::chrono::nanoseconds::max() ? 0ns : offline_timeout_)
    }; ///< 上次心跳时间

    bool terminated_ = false;

public:
    device_base(const std::chrono::nanoseconds offline_timeout);

    /**
     * @brief 判断设备是否离线
     * @return true 设备离线
     * @return false 设备在线
     */
    bool offline() { return offline_timeout_ == 0ms?false:utils::now() - tick_time_ > offline_timeout_; }

    /**
     * @brief 更新心跳时间
     */
    void tick() { tick_time_ = utils::now(); }

    /**
     * @brief 默认的task
     * @details 我们默认每个设备都有自己的task()，但有的设备可能没有，因此在这里提供一个空的task()实现。
     * 
     * @return awaitable<void> 
     */
    awaitable<void> task(){co_return ;}
};

/**
 * @brief 设备概念，约束 task() 等接口。
 */
template<typename T>
concept device = std::is_base_of_v<device_base, T> && roboctrl::owner<T> && requires (T dev) {
    {dev.task()} -> std::same_as<awaitable<void>>;
};

/**
 * @brief 判断多个设备是否离线
 * @param devs 设备列表
 * @return true 有设备离线
 * @return false 所有设备都在线
 */
bool is_offline(auto&... devs)
    requires (requires(decltype(devs) d) {
        { d.offline() } -> std::same_as<bool>;
    } and ...) {
    return (devs.offline() or ...);
}
}
