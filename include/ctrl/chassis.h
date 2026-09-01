#pragma once

#include <chrono>
#include <string_view>

#include "core/async.hpp"
#include "core/logger.h"
#include "device/motor/ref.hpp"
#include "utils/pid.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{
using namespace std::chrono_literals;
class chassis :public utils::singleton_base<chassis>, public logable<chassis> {
public:
    awaitable<void> task();
    std::string desc()const {return "chassis";}

    inline void set_gimbal_yaw(fp32 yaw){gimbal_yaw_ = yaw;}
    inline fp32 gimbal_yaw()const {return gimbal_yaw_;}

    inline void set_velocity(vectorf velocity){velocity_ = velocity;}
    inline vectorf velocity()const {return velocity_;}

    inline void set_rotate_speed(fp32 speed){rotate_speed_ = speed;}
    inline fp32 rotate_speed()const {return rotate_speed_;}

    struct info_type{
        using owner_type = chassis;
        std::string_view left_front_motor {"left_front_motor"};
        std::string_view right_front_motor {"right_front_motor"};
        std::string_view left_rear_motor {"left_rear_motor"};
        std::string_view right_rear_motor {"right_rear_motor"};
        utils::rad_pid::params_type follow_pid {};
        fp32 follow_direction {1.0f};
        fp32 follow_settle_angle {0.005f};
        std::chrono::steady_clock::duration control_time {1ms};
    };

    bool init(const info_type& info);
private:
    awaitable<void> speed_decomposition();
    fp32 resolved_rotate_speed();

    vectorf velocity_{};
    fp32 gimbal_yaw_ {};
    fp32 rotate_speed_ {};
    fp32 max_wheel_speed_ {2.5f};
    fp32 last_rotate_direction_ {};
    fp32 follow_direction_ {1.0f};
    fp32 follow_settle_angle_ {0.005f};
    std::chrono::steady_clock::duration control_time_ {1ms};
    utils::rad_pid follow_pid_;
    device::motor_ref left_front_motor_;
    device::motor_ref right_front_motor_;
    device::motor_ref left_rear_motor_;
    device::motor_ref right_rear_motor_;
};

static_assert(utils::singleton<chassis>);
}
