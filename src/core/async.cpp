#include "core/async.hpp"

using namespace roboctrl::async;

task_context::task_context(){
    log_info("Task Context initiated");
}

void task_context::spawn(task_context::task_type&& task){
    asio::co_spawn(context_, std::move(task), [this](std::exception_ptr error) {
        if (error) {
            handle_unhandled_exception(error, "coroutine");
        }
    });
}

void task_context::set_shutdown_handler(shutdown_handler_type handler)
{
    shutdown_handler_ = std::move(handler);
}

void task_context::run() noexcept {
    log_info("Start running task context");
    while (!context_.stopped()) {
        try {
            context_.run();
            break;
        } catch (...) {
            // A handler submitted directly through the exposed io_context can
            // still throw.  Route it through the same fatal-shutdown path and
            // continue running so that the handler can finish.
            handle_unhandled_exception(
                std::current_exception(), "event-loop handler");
        }
    }
}

void task_context::stop(){
    request_shutdown();
}


bool task_context::init(task_context::info_type _info){
    info_ = _info;
    return true;
}

void task_context::handle_unhandled_exception(
    std::exception_ptr error, std::string_view origin) noexcept
{
    try {
        if (error) {
            std::rethrow_exception(error);
        }
    } catch (const std::exception& exception) {
        try {
            log_error("unhandled exception from {}: {}", origin, exception.what());
        } catch (...) {
        }
    } catch (...) {
        try {
            log_error("unhandled exception from {}: unknown exception", origin);
        } catch (...) {
        }
    }

    failed_ = true;
    request_shutdown();
}

void task_context::request_shutdown() noexcept
{
    if (shutdown_started_) {
        return;
    }
    shutdown_started_ = true;

    if (!shutdown_handler_) {
        // Applications with actuators must install a bounded safe-stop
        // handler before run().  Without one, stopping immediately is the only
        // generic action Core can take without depending on control layers.
        context_.stop();
        return;
    }

    try {
        auto shutdown_task = shutdown_handler_();
        asio::co_spawn(
            context_, std::move(shutdown_task),
            [this](std::exception_ptr shutdown_error) {
                finish_shutdown(shutdown_error);
            });
    } catch (...) {
        finish_shutdown(std::current_exception());
    }
}

void task_context::finish_shutdown(std::exception_ptr error) noexcept
{
    if (error) {
        try {
            std::rethrow_exception(error);
        } catch (const std::exception& exception) {
            try {
                log_error("shutdown handler failed: {}", exception.what());
            } catch (...) {
            }
        } catch (...) {
            try {
                log_error("shutdown handler failed with an unknown exception");
            } catch (...) {
            }
        }
    }
    context_.stop();
}
