#include "device/chassis/gkd_sentry_chassis.hpp"
#include "core/async.hpp"
#include "utils/kinematics/mecanum.hpp"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/utils.hpp"

using namespace roboctrl::device;

ROBOCTRL_REGISTER_CHASSIS("device.gkd_sentry_chassis.v1", roboctrl::device::gkd_sentry_chassis);
ROBOCTRL_REGISTER_CHASSIS("device.standard_mecanum_chassis.v1", roboctrl::device::gkd_sentry_chassis);
ROBOCTRL_REGISTER_CHASSIS("ctrl.standard_mecanum_chassis.v1", roboctrl::device::gkd_sentry_chassis);

roboctrl::awaitable<void> gkd_sentry_chassis::task()
{
    while(true){
        co_await speed_decomposition();
        co_await roboctrl::wait_for(control_time_);
    }
}

bool gkd_sentry_chassis::init(const gkd_sentry_chassis::info_type& info){
    left_front_motor_ = &roboctrl::get<dji_motor>(info.left_front_motor);
    right_front_motor_ = &roboctrl::get<dji_motor>(info.right_front_motor);
    left_rear_motor_ = &roboctrl::get<dji_motor>(info.left_rear_motor);
    right_rear_motor_ = &roboctrl::get<dji_motor>(info.right_rear_motor);
    control_time_ = info.control_time;
    max_rotate_speed_ = info.max_rotate_speed;
    log_info("Chassis initiated");
    roboctrl::spawn(task());
    return true;
}

roboctrl::awaitable<void> gkd_sentry_chassis::speed_decomposition(){
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
