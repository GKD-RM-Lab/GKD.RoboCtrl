/**
 * @file logger.h
 * @author Junity
 * @brief 用于日志输出的组件。
 * @details 提供日志等级控制和格式化日志输出功能，支持Debug、Info、Warn和Error四个等级。
 * Logger类是提供日志基础功能的单例类，用于管理全局日志设置和输出。
 * logable类是一个辅助基类，通过CRTP模式为派生类提供日志输出功能，要求派生类实现desc()方法以提供描述信息。
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <atomic>
#include <format>
#include <mutex>
#include <string_view>
#include <utility>

#include "utils/singleton.hpp"
#include "core/multiton.hpp"

namespace roboctrl{
/**
 * @brief 日志模块
 * @details 
 * 这是电控的日志模块，提供了日志功能和日志等级控制。日志等级分为debug,info,warn和error，每种日志的颜色都不一样，
 * 可以在 @ref src/core/logger.cpp "src/core/logger.cpp" 定义的常量中看到。
 *
 * 每一行日志都对应了一个role，表示这行日志的发送者是谁。对于实现了desc() 并继承自logable<T> 的类，
 * 
 */
namespace log{

enum log_level { Debug = 0, Info, Warn, Error };

/**
 * @brief 日志类
 * 
 */
class logger: public utils::singleton_base<logger> {
public:
    /**
    * @brief 设置日志等级
    * 
    * @param level 日志等级
    */
    static void set_level(log_level level);

    /**
    * @brief 获取日志等级
    * 
    * @return log_level 日志等级
    */
    static log_level level();

    friend class singleton_base<logger>;

    /**
     * @brief 日志接口
     * 
     * @tparam Args 参数类型
     * @param level 日志等级
     * @param role 日志角色(描述这行日志是由哪个模块/类/函数产生的)
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */

    template <typename... Args>
    void log(log_level level,
             std::string_view role,
             std::format_string<Args...> fmt,
             Args &&...args) {
        if (static_cast<int>(level) < static_cast<int>(_level.load())) {
            return;
        }
        const auto message = std::format(fmt, std::forward<Args>(args)...);
        log_impl(level, role, message);
    }

    /**
     * @brief 打印debug日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */

    template <typename... Args>
    void log_debug(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Debug, "", fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 打印info日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */

    template <typename... Args>
    void log_info(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Info, "", fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 打印warn日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */

    template <typename... Args>
    void log_warn(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Warn, "", fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 打印error日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */

    template <typename... Args>
    void log_error(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Error, "", fmt, std::forward<Args>(args)...);
    }

    static void set_filter(const std::string filter);

private:
    logger() = default;

    void log_impl(log_level level, std::string_view role, std::string_view message);
    static std::string_view level_to_string(log_level level);

    mutable std::mutex _mutex;
    std::atomic<log_level> _level{log_level::Info};
    std::string filter_;
};

#define LOG_LOGGER_CALL(level, role, fmt, ...)                                                   \
    ::roboctrl::logger::instance().log(level, role, fmt __VA_OPT__(, ) __VA_ARGS__)

#define GET_ROLE (std::string(__FILE__) + ":" + std::to_string(__LINE__) + ":" + __FUNCTION__)

#define LOG_DEBUG(fmt, ...) LOG_LOGGER_CALL(::roboctrl::log_level::Debug, GET_ROLE, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_INFO(fmt, ...) LOG_LOGGER_CALL(::roboctrl::log_level::Info, GET_ROLE, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_WARN(fmt, ...) LOG_LOGGER_CALL(::roboctrl::log_level::Warn, GET_ROLE, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_ERROR(fmt, ...) LOG_LOGGER_CALL(::roboctrl::log_level::Error, GET_ROLE, fmt __VA_OPT__(, ) __VA_ARGS__)

/**
 * @brief 用于提供日志功能的辅助基类
 * 通过CRTP模式实现，要求派生类实现desc()方法以提供描述信息，在继承时传入派生类自身类型作为模板参数。
 * 继承这个类后，派生类可以方便地使用log_debug、log_info、log_warn和log_error方法来记录日志，这些方法会自动包含类的描述信息。
 * 示例：
 * ```cpp
 * class MyClass : public logable<MyClass> {
 * public:
 *     std::string desc() const {
 *         return "MyClassInstance";
 *     }
 *     void do_something() {
 *         log_info("Doing something with value: {}", 42);
 *     }
 * };
 * ```
 * 
 * @tparam T 要添加日志输出功能的类
 */
template <typename T>
class logable{
protected:
    /**
     * @brief 输出日志
     * 
     * @tparam Args 参数类型
     * @param level 日志等级
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */
    template <typename... Args>
    void log(log_level level, std::format_string<Args...> fmt, Args &&...args) const
     requires descable<T>{
        logger::instance().log(level, static_cast<const T*>(this)->desc(), fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 输出debug日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */
    template <typename... Args>
    void log_debug(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Debug, fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 输出info日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */
    template <typename... Args>
    void log_info(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Info, fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 输出warn日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */
    template <typename... Args>
    void log_warn(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Warn, fmt, std::forward<Args>(args)...);
    }

    /**
     * @brief 输出error日志
     * 
     * @tparam Args 
     * @param fmt 日志格式，参考std::format
     * @param args 日志参数，参考std::format
     */
    template <typename... Args>
    void log_error(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Error, fmt, std::forward<Args>(args)...);
    }
};
}

using namespace log;

}