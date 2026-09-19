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
#include <vector>

#include <iostream>
#include <signal.h>

using namespace std::chrono_literals;
using namespace roboctrl;
using namespace roboctrl::log;

namespace {

constexpr auto shutdown_grace_period = 20ms;

roboctrl::awaitable<void> safely_stop_robot()
{
    auto& robot = roboctrl::get<ctrl::robot>();
    robot.set_state(ctrl::robot_state::NoForce);

    // Do not rely only on the periodic group coroutine: that coroutine may be
    // the task that just failed.  Explicitly enqueue one disabled snapshot for
    // every group; CAN's latest-value queue replaces older pending commands.
    std::vector<device::dji_motor_group*> groups;
    roboctrl::for_each_instance<device::dji_motor_group>(
        [&groups](device::dji_motor_group& group) { groups.push_back(&group); });
    for (auto* group : groups) {
        try {
            co_await group->flush_commands_once();
        } catch (const std::exception& error) {
            logger::instance().log_error(
                "failed to enqueue emergency zero output for {}: {}",
                group->desc(), error.what());
        } catch (...) {
            logger::instance().log_error(
                "failed to enqueue emergency zero output for {}",
                group->desc());
        }
    }

    // Keep the event loop alive for a bounded window so the 1 ms DJI group
    // task can enqueue zero-current frames before task_context stops it.
    co_await roboctrl::wait_for(shutdown_grace_period);
}

} // namespace

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

    cxxopts::Options options(
        "roboctrl", "Roboctrl with runtime-selected robot profile");
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

    async::set_shutdown_handler(safely_stop_robot);

    asio::signal_set shutdown_signals{async::io_context(), SIGINT, SIGTERM};
    int received_signal = 0;
    shutdown_signals.async_wait(
        [&received_signal](const auto& error, int signal_number) {
            if (error) {
                return;
            }
            received_signal = signal_number;
            LOG_WARN("Received signal {}; requesting safe shutdown", signal_number);
            async::request_shutdown();
        });

    async::run();
    if (async::failed()) {
        return 1;
    }
    return received_signal == 0 ? 0 : 128 + received_signal;
}
