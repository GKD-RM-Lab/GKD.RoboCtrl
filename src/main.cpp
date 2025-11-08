#include "asio/steady_timer.hpp"
#include "core/logger.h"
#include "core/task_context.hpp"
#include <cxxopts.hpp>
#include "io/can.h"
#include "io/serial.h"
#include <iostream>
#include "ctrl/robot.h"
using namespace std::chrono_literals;

bool init(){

}

int main(int argc,char** argv){

#ifdef DEBUG
    roboctrl::logger::set_level(roboctrl::log_level::Debug);
#else
    roboctrl::logger::set_level(roboctrl::log_level::Info);
#endif

    cxxopts::Options options("roboctrl", "A robot controller");
    options.add_options()
        ("h,help", "Print help")
        ("lg,log-level", "Log level", cxxopts::value<std::string>()->default_value("info"));
    
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::print("{:s}\n", options.help());
        return 0;
    }

    if(result.count("log-level")){
        if(result["log-level"].as<std::string>() == "debug")
            roboctrl::logger::set_level(roboctrl::log_level::Debug);
        else if(result["log-level"].as<std::string>() == "info")
            roboctrl::logger::set_level(roboctrl::log_level::Info);
        else if(result["log-level"].as<std::string>() == "warn")
            roboctrl::logger::set_level(roboctrl::log_level::Warn);
        else if(result["log-level"].as<std::string>() == "error")
            roboctrl::logger::set_level(roboctrl::log_level::Error);
        else{
            std::print("Invalid log level\n");
            return 1;
        }
    }

    roboctrl::init(
        roboctrl::task_context::info_type::make()
    );

    auto& tc = roboctrl::get<roboctrl::task_context>(0);


    tc.spawn([&tc]() -> roboctrl::awaitable<void> {
        int i = 0;
        while(true){
            LOG_INFO("test : {}",i++);
            co_await roboctrl::wait_for(1s);
        }
    }());

    tc.run();
}