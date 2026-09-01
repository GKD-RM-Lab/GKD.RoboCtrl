#include "ctrl/chassis.h"
#include "ctrl/chassis_kinematics.hpp"
#include "core/async.hpp"
#include "ctrl/gimbal.h"
#include "ctrl/robot.h"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/utils.hpp"

using namespace roboctrl::ctrl;
using namespace roboctrl::device;

roboctrl::awaitable<void> chassis::task()
{
    while(true){
        co_await speed_decomposition();
        co_await roboctrl::wait_for(1ms);
    }
}

bool chassis::init(const chassis::info_type& info){
    left_front_motor_ = motor_ref::from<dji_motor>("left_front_motor");
    right_front_motor_ = motor_ref::from<dji_motor>("right_front_motor");
    left_rear_motor_ = motor_ref::from<dji_motor>("left_rear_motor");
    right_rear_motor_ = motor_ref::from<dji_motor>("right_rear_motor");
    log_info("Chassis initiated");
    roboctrl::spawn(task());
    return true;
}

roboctrl::awaitable<void> chassis::speed_decomposition(){
    if (roboctrl::get<robot>().state() == robot_state::NoForce) {
        co_await left_front_motor_.set(0.0f);
        co_await right_front_motor_.set(0.0f);
        co_await left_rear_motor_.set(0.0f);
        co_await right_rear_motor_.set(0.0f);
        co_return;
    }

    const auto wheels = mecanum_wheel_speeds(
        velocity_, gimbal_yaw_, rotate_speed_, max_wheel_speed_);

    log_debug("left_front_motor : {}",wheels.left_front);
    log_debug("right_front_motor : {}",-wheels.right_front);
    log_debug("left_rear_motor : {}",wheels.left_rear);
    log_debug("right_rear_motor : {}",-wheels.right_rear);

    co_await left_front_motor_.set(wheels.left_front);
    co_await right_front_motor_.set(-wheels.right_front);
    co_await left_rear_motor_.set(wheels.left_rear);
    co_await right_rear_motor_.set(-wheels.right_rear);
}
