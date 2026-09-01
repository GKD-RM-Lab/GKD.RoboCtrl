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
        co_await roboctrl::wait_for(control_time_);
    }
}

bool chassis::init(const chassis::info_type& info){
    left_front_motor_ = motor_ref::from<dji_motor>(info.left_front_motor);
    right_front_motor_ = motor_ref::from<dji_motor>(info.right_front_motor);
    left_rear_motor_ = motor_ref::from<dji_motor>(info.left_rear_motor);
    right_rear_motor_ = motor_ref::from<dji_motor>(info.right_rear_motor);
    follow_pid_ = utils::rad_pid{info.follow_pid};
    follow_pid_.set_target(0.0f);
    follow_direction_ = info.follow_direction;
    follow_settle_angle_ = info.follow_settle_angle;
    control_time_ = info.control_time;
    log_info("Chassis initiated");
    roboctrl::spawn(task());
    return true;
}

roboctrl::fp32 chassis::resolved_rotate_speed() {
    if (rotate_speed_ != 0.0f) {
        last_rotate_direction_ = std::copysign(1.0f, rotate_speed_);
        return rotate_speed_;
    }
    if (last_rotate_direction_ != 0.0f && std::fabs(gimbal_yaw_) > follow_settle_angle_) {
        return last_rotate_direction_;
    }
    last_rotate_direction_ = 0.0f;
    follow_pid_.update(gimbal_yaw_);
    return follow_pid_.state() * follow_direction_;
}

roboctrl::awaitable<void> chassis::speed_decomposition(){
    if (roboctrl::get<robot>().state() == robot_state::NoForce) {
        last_rotate_direction_ = 0.0f;
        follow_pid_.clean();
        follow_pid_.set_target(0.0f);
        co_await left_front_motor_.set(0.0f);
        co_await right_front_motor_.set(0.0f);
        co_await left_rear_motor_.set(0.0f);
        co_await right_rear_motor_.set(0.0f);
        co_return;
    }

    const auto wheels = mecanum_wheel_speeds(
        velocity_, gimbal_yaw_, resolved_rotate_speed(), max_wheel_speed_);

    log_debug("left_front_motor : {}",wheels.left_front);
    log_debug("right_front_motor : {}",-wheels.right_front);
    log_debug("left_rear_motor : {}",wheels.left_rear);
    log_debug("right_rear_motor : {}",-wheels.right_rear);

    co_await left_front_motor_.set(wheels.left_front);
    co_await right_front_motor_.set(-wheels.right_front);
    co_await left_rear_motor_.set(wheels.left_rear);
    co_await right_rear_motor_.set(-wheels.right_rear);
}
