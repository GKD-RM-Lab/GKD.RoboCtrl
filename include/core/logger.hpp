#pragma once

#include <atomic>
#include <format>
#include <mutex>
#include <string_view>
#include <utility>

#include "utils/singleton.hpp"
#include "core/multiton.hpp"

namespace roboctrl{

enum class log_level { Debug = 0, Info, Warn, Error };

class logger: public utils::singleton<logger> {
public:
    void set_level(log_level level);
    log_level level() const;

    friend class singleton<logger>;

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

    template <typename... Args>
    void log_info(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Info, "", fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_warn(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Warn, "", fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_error(std::format_string<Args...> fmt, Args &&...args) {
        log(log_level::Error, "", fmt, std::forward<Args>(args)...);
    }

    void progress(double ratio, std::size_t completed, std::size_t total);

private:
    logger() = default;

    void log_impl(log_level level, std::string_view role, std::string_view message);
    static std::string_view level_to_string(log_level level);

    mutable std::mutex _mutex;
    std::atomic<log_level> _level{log_level::Info};
};

#define LOG_LOGGER_CALL(level, role, fmt, ...)                                                   \
    logger::instance().log(level, role, fmt __VA_OPT__(, ) __VA_ARGS__)

#define LOG_DEBUG(role, fmt, ...) LOG_LOGGER_CALL(log_level::Debug, role, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_INFO(role, fmt, ...) LOG_LOGGER_CALL(log_level::Info, role, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_WARN(role, fmt, ...) LOG_LOGGER_CALL(log_level::Warn, role, fmt __VA_OPT__(, ) __VA_ARGS__)
#define LOG_ERROR(role, fmt, ...) LOG_LOGGER_CALL(log_level::Error, role, fmt __VA_OPT__(, ) __VA_ARGS__)

template <owner T>
class logable {
protected:
    template <typename... Args>
    void log(log_level level, std::format_string<Args...> fmt, Args &&...args) const {
        logger::instance().log(level, static_cast<const T*>(this)->desc(), fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_info(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Info, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_warn(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Warn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log_error(std::format_string<Args...> fmt, Args &&...args) const {
        log(log_level::Error, fmt, std::forward<Args>(args)...);
    }
};

}