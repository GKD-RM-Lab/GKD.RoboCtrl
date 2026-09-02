#include "device/chassis.hpp"
#include "core/async.hpp"
#include "utils/kinematics/mecanum.hpp"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/utils.hpp"
#include <any>
#include <mutex>
#include <unordered_map>
#include <utility>

using namespace roboctrl::device;

ROBOCTRL_REGISTER_CHASSIS("device.standard_mecanum_chassis.v1", roboctrl::device::mecanum_chassis);
ROBOCTRL_REGISTER_CHASSIS("ctrl.standard_mecanum_chassis.v1", roboctrl::device::mecanum_chassis);

roboctrl::awaitable<void> mecanum_chassis::task()
{
    while(true){
        co_await speed_decomposition();
        co_await roboctrl::wait_for(control_time_);
    }
}

bool mecanum_chassis::init(const mecanum_chassis::info_type& info){
    left_front_motor_ = &roboctrl::get<dji_motor>(info.left_front_motor);
    right_front_motor_ = &roboctrl::get<dji_motor>(info.right_front_motor);
    left_rear_motor_ = &roboctrl::get<dji_motor>(info.left_rear_motor);
    right_rear_motor_ = &roboctrl::get<dji_motor>(info.right_rear_motor);
    control_time_ = info.control_time;
    log_info("Chassis initiated");
    roboctrl::spawn(task());
    return true;
}

namespace {
std::unordered_map<std::string, roboctrl::device::chassis_registry::factory>& chassis_factories() {
    static std::unordered_map<std::string, roboctrl::device::chassis_registry::factory> value;
    return value;
}
std::mutex& chassis_factories_mutex() { static std::mutex mutex; return mutex; }
roboctrl::device::chassis_base*& current_chassis() { static roboctrl::device::chassis_base* value = nullptr; return value; }
}

bool chassis_registry::register_type(std::string type, factory creator) {
    std::lock_guard lock{chassis_factories_mutex()};
    return chassis_factories().emplace(std::move(type), std::move(creator)).second;
}

chassis_base* chassis_registry::current() { return current_chassis(); }

chassis_base* chassis_registry::create(std::string_view type, const std::any& info) {
    factory creator;
    {
        std::lock_guard lock{chassis_factories_mutex()};
        auto it = chassis_factories().find(std::string{type});
        if (it == chassis_factories().end()) return nullptr;
        creator = it->second;
    }
    return creator(info);
}

bool chassis_registry::init(std::string_view type, const mecanum_chassis::info_type& info) {
    return init(type, std::any{info});
}

bool chassis_registry::init(std::string_view type, const std::any& info) {
    auto* result = create(type, info);
    current_chassis() = result;
    return result != nullptr;
}

roboctrl::awaitable<void> mecanum_chassis::speed_decomposition(){
    if (!enabled_) {
        co_await left_front_motor_->set(0.0f);
        co_await right_front_motor_->set(0.0f);
        co_await left_rear_motor_->set(0.0f);
        co_await right_rear_motor_->set(0.0f);
        co_return;
    }

    const auto wheels = utils::kinematics::inverse_mecanum(
        velocity_, rotate_speed_, max_wheel_speed_);

    log_debug("left_front_motor : {}",wheels.left_front);
    log_debug("right_front_motor : {}",-wheels.right_front);
    log_debug("left_rear_motor : {}",wheels.left_rear);
    log_debug("right_rear_motor : {}",-wheels.right_rear);

    co_await left_front_motor_->set(wheels.left_front);
    co_await right_front_motor_->set(-wheels.right_front);
    co_await left_rear_motor_->set(wheels.left_rear);
    co_await right_rear_motor_->set(-wheels.right_rear);
}
