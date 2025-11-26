#include "config/config.hpp"
#include "core/logger.h"
#include "core/async.hpp"
#include "core/multiton.hpp"
#include "ctrl/robot.h"
#include "device/controlpad.h"
#include "device/imu/serial_imu.hpp"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "io/can.h"
#include "io/serial.h"
#include <concepts>
#include <cxxopts.hpp>
#include <print>
#include <stdexcept>

#include <iostream>
#include <signal.h>

using namespace std::chrono_literals;
using namespace roboctrl;
using namespace roboctrl::log;

#define check_init(conf)        \
    if(!roboctrl::init(conf))   \
        return false

static bool init(){
    try{
        check_init(config::cans);
        check_init(config::serials);
        check_init(config::dji_motors);
        check_init(config::control_pad);
        // check_init(config::imu);
        // check_init(config::robot);
    }
    catch(std::runtime_error e){
        std::println("exception : {}",e.what());
        return false;
    }

    return true;
}

#undef check_init

int main(int argc,char** argv){
#ifdef DEBUG
    roboctrl::logger::set_level(roboctrl::log_level::Debug);
#else
    logger::set_level(log::Info);
#endif

    cxxopts::Options options("roboctrl", "Roboctrl for " TYPE_STR);
    options.add_options()
        ("h,help", "Print help")
        ("l,log", "Log level", cxxopts::value<std::string>()->default_value("info"))
        ("f,filter","Filter for logger",cxxopts::value<std::string>()->default_value(""));
    
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::print("{:s}\n", options.help());
        return 0;
    }

    if(result.count("log")){
        if(result["log"].as<std::string>() == "debug")
            logger::set_level(log::Debug);
        else if(result["log"].as<std::string>() == "info")
            logger::set_level(log::Info);
        else if(result["log"].as<std::string>() == "warn")
            logger::set_level(log::Warn);
        else if(result["log"].as<std::string>() == "error")
            logger::set_level(log::Error);
        else{
            std::print("Invalid log level\n");
            return 1;
        }
    }

    if(result.count("filter")){
        logger::set_filter(result["filter"].as<std::string>());
    }

    if(!::init()){
        std::println("Initiation failed");
        return -1;
    }

    LOG_INFO("Initiation finished.");

    roboctrl::get<ctrl::robot>().set_velocity(0.1,0.1);

    async::run();
}
