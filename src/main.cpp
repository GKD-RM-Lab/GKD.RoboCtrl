#include "config/runtime.hpp"
#include "config/validate.hpp"
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
#include <filesystem>
#include <print>
#include <span>
#include <stdexcept>

#include <iostream>
#include <signal.h>

using namespace std::chrono_literals;
using namespace roboctrl;
using namespace roboctrl::log;

#define check_init(conf)        \
    if(!roboctrl::init(conf))   \
        return false

static bool initialize_system(const config::runtime_config& system_config){
    try{
        config::validate_configuration(
            std::span{system_config.cans},
            std::span{system_config.serials},
            std::span{system_config.dji_motors},
            system_config.control_pad,
            system_config.imu,
            system_config.robot
        );

        check_init(std::span{system_config.cans});
        check_init(std::span{system_config.serials});
        check_init(std::span{system_config.dji_motors});
        check_init(system_config.control_pad);
        check_init(system_config.imu);

        roboctrl::connect_all<device::dji_motor>();
        check_init(system_config.robot);

        roboctrl::start_all<io::can>();
        roboctrl::start_all<io::serial>();
        roboctrl::start_all<device::dji_motor_group>();
        roboctrl::start_all<device::dji_motor>();
    }
    catch(const std::exception& e){
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
        ("f,filter","Filter for logger",cxxopts::value<std::string>()->default_value(""))
        ("c,config", "YAML or JSON configuration file", cxxopts::value<std::string>());
    
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

    const std::filesystem::path config_path = result.count("config")
        ? std::filesystem::path{result["config"].as<std::string>()}
        : config::default_configuration_path();
    config::print_available_configurations(std::cout, config_path.parent_path());

    auto loaded_config = config::load_configuration(config_path);
    if (!loaded_config) {
        std::println("Configuration error: {}", loaded_config.error());
        return 1;
    }

    auto system_config = std::move(*loaded_config);
    if(!initialize_system(system_config)){
        std::println("Initiation failed");
        return -1;
    }

    LOG_INFO("Initiation finished.");

    async::run();
}
