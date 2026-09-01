#include "core/async.hpp"

using namespace roboctrl::async;

task_context::task_context(){
    log_info("Task Context initiated");
}

void task_context::spawn(task_context::task_type&& task){
    asio::co_spawn(context_, std::move(task), [this](std::exception_ptr error) {
        if (!error) {
            return;
        }
        try {
            std::rethrow_exception(error);
        } catch (const std::exception& e) {
            log_error("asynchronous task failed: {}", e.what());
        } catch (...) {
            log_error("asynchronous task failed with an unknown exception");
        }
        context_.stop();
    });
}

void task_context::run(){
    log_info("Start running task context");
    context_.run();
}

void task_context::stop(){
    log_info("Stop running task context");
    context_.stop();
}


bool task_context::init(task_context::info_type _info){
    return true;
}
