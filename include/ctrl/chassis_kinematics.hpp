#pragma once

#include <algorithm>
#include <cmath>

#include "utils/utils.hpp"

namespace roboctrl::ctrl {

struct wheel_speeds {
    fp32 left_front {};
    fp32 right_front {};
    fp32 left_rear {};
    fp32 right_rear {};
};

inline wheel_speeds mecanum_wheel_speeds(
    vectorf velocity,
    fp32 gimbal_yaw,
    fp32 rotate_speed,
    fp32 max_wheel_speed)
{
    const fp32 sin_yaw = std::sin(gimbal_yaw);
    const fp32 cos_yaw = std::cos(gimbal_yaw);
    const fp32 vx =  cos_yaw * velocity.x + sin_yaw * velocity.y;
    const fp32 vy = -sin_yaw * velocity.x + cos_yaw * velocity.y;

    wheel_speeds result{
        .left_front = vx - vy - rotate_speed,
        .right_front = vx + vy + rotate_speed,
        .left_rear = vx + vy - rotate_speed,
        .right_rear = vx - vy + rotate_speed
    };

    const fp32 peak = std::max({
        std::fabs(result.left_front),
        std::fabs(result.right_front),
        std::fabs(result.left_rear),
        std::fabs(result.right_rear)
    });
    if (peak > max_wheel_speed && peak > 0.0f) {
        const fp32 factor = max_wheel_speed / peak;
        result.left_front *= factor;
        result.right_front *= factor;
        result.left_rear *= factor;
        result.right_rear *= factor;
    }
    return result;
}

} // namespace roboctrl::ctrl
