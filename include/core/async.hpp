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
#include <exception>
#include <format>
#include <functional>
#include <string_view>
#include <tuple>
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
 * @details 电控代码目前被实现为 **单线程异步** ，即整个逻辑异步运行在单线程中。它避免了多线程同时访问内存，
 * 但协程仍会在 `co_await` 处交错，因此共享状态、对象生命周期和异步写操作仍需显式管理。
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
using awaitable = asio::awaitable<T>;

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
    using shutdown_handler_type = std::function<task_type()>;

    /// @internal
    struct info_type{
        using owner_type = task_context;
    };

    explicit task_context();

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
    void spawn(task_type&& task);

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
            [this, fn = std::forward<Fn>(fn),
            args = std::make_tuple(std::decay_t<Args>(std::forward<Args>(args))...)]() mutable {
                try {
                    std::apply(std::move(fn), std::move(args));
                } catch (...) {
                    handle_unhandled_exception(
                        std::current_exception(), "posted task");
                }
            });
    }

    /**
     * @brief 注册异常或外部请求共用的一次性安全停机任务。
     *
     * 首个越过 spawn/post/run 边界的异常会启动该任务。事件循环会继续
     * 运行，直到任务完成后才停止，从而允许上层先撤销执行器使能并等待
     * 有界的零输出刷新窗口。停机任务自身不得无限等待。
     */
    void set_shutdown_handler(shutdown_handler_type handler);

    /**
     * @brief 请求一次安全停机。
     *
     * 首次请求会运行已注册的停机任务，待其完成后停止事件循环。重复请求
     * 不会重复运行停机任务。
     */
    void request_shutdown() noexcept;

    /**
     * @brief 开始运行任务上下文。
     */
    void run() noexcept;

    void stop();

    /** @brief 是否观察到过未处理的异步异常。 */
    bool failed() const noexcept { return failed_; }

    /** @brief 是否已经进入一次性停机流程。 */
    bool shutdown_requested() const noexcept { return shutdown_started_; }

    /// @brief 初始化 task_context
    bool init(info_type _info);

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
    void handle_unhandled_exception(std::exception_ptr error,
                                    std::string_view origin) noexcept;
    void finish_shutdown(std::exception_ptr error) noexcept;

    asio::io_context context_;
    info_type info_;
    shutdown_handler_type shutdown_handler_;
    bool failed_ {false};
    bool shutdown_started_ {false};
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
inline auto spawn(task_context::task_type&& task){
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

/** @brief 查询全局任务上下文是否因未处理异常进入过安全停机。 */
inline bool failed(){
    return roboctrl::get<task_context>().failed();
}

/** @brief 查询全局任务上下文是否已经进入停机流程。 */
inline bool shutdown_requested(){
    return roboctrl::get<task_context>().shutdown_requested();
}

/** @brief 设置全局任务上下文的一次性安全停机任务。 */
inline void set_shutdown_handler(task_context::shutdown_handler_type handler){
    roboctrl::get<task_context>().set_shutdown_handler(std::move(handler));
}

/** @brief 请求运行全局任务上下文的安全停机任务。 */
inline void request_shutdown(){
    roboctrl::get<task_context>().request_shutdown();
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
    auto ex = roboctrl::get<task_context>().get_executor();
    asio::steady_timer timer(ex, duration);
    co_await timer.async_wait(asio::use_awaitable);
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
