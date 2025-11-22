/**
 * @file async.hpp
 * @author Junity
 * @brief 异步任务上下文组件。
 * @details 提供基于Asio的异步任务执行环境，支持协程任务的调度和执行。
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#include <chrono>
#include <format>
#include <utility>
#include <asio.hpp>
#include <asio/awaitable.hpp>
#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <asio/use_awaitable.hpp>

#include "multiton.hpp"
#include "core/logger.h"
#include "utils/singleton.hpp"

/**
 * @brief 异步上下文相关功能
 * @details 电控代码目前被实现为 **单线程异步** ，即整个逻辑异步运行在单线程中。这样的优势是可以不用在意线程同步的问题，并且
 * 使得涉及到 IO 的代码可以更加优雅。为了上手C++中的异步编程，你可以先看看Python中的异步编程是怎么样的。
 * 
 * 异步上下文被设计成一个单例类，可以通过 roboctrl::get<task_context>() 获取，这是因为目前应该不需要多个异步任务上下文。
 */
namespace roboctrl::async{

/**
 * @brief 协程任务类型。
 * 
 * @tparam T 返回值类型
 */
template<typename T = void>
using awaitable = asio::awaitable<void>;

using duration = std::chrono::steady_clock::duration;

/**
 * @brief 异步任务上下文类。
 * @details 提供基于Asio的异步任务执行环境，支持协程任务的调度和执行。
 * 
 */
class task_context :
 public utils::singleton_base<task_context>,public logable<task_context>{
public:
    using task_type = awaitable<>;

    /// @internal
    struct info_type{
        using owner_type = task_context;
    };

    inline explicit task_context() {}

    /**
     * @brief 添加一个协程任务到上下文中执行。
     * 
     * @param task 任务
     * @details 示例：
     *
     * ```cpp
     * roboctrl::spawn([]() -> roboctrl::awaitable<void>{
     *     while(true){
     *         std::print("hello");
     *         co_await roboctrl::wait_for(1s);
     *     }
     * }());
     * ```
     */
    inline void spawn(task_type&& task){
        asio::co_spawn(context_,std::move(task),asio::detached);
    }

    /**
     * @brief 添加一个任务到上下文中执行。
     * 
     * @tparam Fn 任务类型
     * @tparam Args 任务参数类型
     * @param fn 任务
     * @param args 任务参数
     * @details 示例：
     *
     * ```cpp
     * roboctrl::post([](int a,int b){
     *     std::print("{} + {} = {}\n",a,b,a+b);
     * },1,2);
     * ```
     */
    template<typename Fn, typename... Args>
    inline void post(Fn&& fn, Args&&... args) {
        asio::post(context_,
            [fn = std::forward<Fn>(fn),
            args = std::make_tuple(std::decay_t<Args>(std::forward<Args>(args))...)]() mutable {
                return std::apply(std::move(fn), std::move(args));
            });
    }

    /**
     * @brief 开始运行任务上下文。
     */
    inline void run(){
        log_info("Start running task context");
        context_.run();
    }

    inline void stop(){
        log_info("Stop running task context");
        context_.stop();
    }

    /// @brief 初始化 task_context
    inline bool init(info_type _info){return true;}

    /// @brief task_context的描述
    inline std::string desc()const{
        return std::format("async task context");
    }

    /**
     * @brief 获取asio的executor。
     */
    inline auto get_executor(){
        return context_.get_executor();
    }

    /**
     * @brief 获取asio的io_context。
     */
    inline auto asio_context() -> asio::io_context&{
        return context_;
    }
    
private:
    asio::io_context context_;
    info_type info_;
};

static_assert(utils::singleton_info<task_context::info_type>);
static_assert(utils::singleton<task_context>);

/**
 * @brief 添加一个协程任务到全局任务上下文中执行。
 * 
 * @param task 任务
 * @details 示例：
 *
 * ```cpp
 * roboctrl::spawn([]() -> roboctrl::awaitable<void>{})
 * ```
 */
inline auto spawn(task_context::task_type task){
    roboctrl::get<task_context>().spawn(std::forward<task_context::task_type>(task));
}

/**
 * @brief 添加一个任务到全局任务上下文中执行。
 * 
 * @tparam Args 任务参数类型
 * @param args 任务参数
 * @details 示例：
 *
 * ```cpp
 * roboctrl::post([](int a,int b){std::print("{} + {} = {}\n",a,b,a+b);},1,2);
 * ```
 */
template<typename ...Args>
inline auto post(Args&&... args)
{
    roboctrl::get<task_context>().post(std::forward<Args>(args)...);
}

/**
 * @brief 运行全局任务上下文。
 * 
 */
inline void run(){
    roboctrl::get<task_context>().run();
}

/**
 * @brief 协程任务等待。用于让出当前函数的执行权。
 * @details 示例：
 *
 * ```cpp
 * co_await yield();
 * ```
 */
inline awaitable<void> yield(){
    co_await asio::post(asio::use_awaitable);
}

/**
 * @brief 协程任务等待。
 * 
 * @param duration 等待时长
 * @details 示例：
 *
 * ```cpp
 * co_await wait_for(1ms);
 * ```
 */
inline awaitable<void> wait_for(const duration& duration){
    auto exector = roboctrl::get<task_context>().get_executor();
    co_await asio::steady_timer(exector,duration).async_wait(asio::use_awaitable);
}

/**
 * @brief 获取全局任务上下文的executor。
 */
inline auto executor(){
    return roboctrl::get<task_context>().get_executor();
}

/**
 * @brief 获取全局任务上下文的io_context。
 */
inline auto& io_context(){
    return roboctrl::get<task_context>().asio_context();
}

inline void stop(){
    roboctrl::get<task_context>().stop();
}
}

namespace roboctrl{
    using namespace async;
}
