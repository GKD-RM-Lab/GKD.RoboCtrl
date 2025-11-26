#include "core/async.hpp"

using namespace roboctrl::async;

task_context::task_context(){
    log_info("Task Context initiated");
}

void task_context::spawn(task_context::task_type&& task){
    asio::co_spawn(context_,std::move(task),asio::detached);
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