#pragma once

#include <algorithm>
#include <cmath>

#include "utils/utils.hpp"

namespace roboctrl::utils::kinematics {

struct mecanum_wheel_speeds {
    fp32 left_front {};
    fp32 right_front {};
    fp32 left_rear {};
    fp32 right_rear {};
};

inline mecanum_wheel_speeds inverse_mecanum(
    vectorf velocity, fp32 rotate_speed, fp32 max_wheel_speed)
{
    mecanum_wheel_speeds result{
        .left_front = velocity.x - velocity.y - rotate_speed,
        .right_front = velocity.x + velocity.y + rotate_speed,
        .left_rear = velocity.x + velocity.y - rotate_speed,
        .right_rear = velocity.x - velocity.y + rotate_speed};
    const fp32 peak = std::max({std::fabs(result.left_front), std::fabs(result.right_front),
                                std::fabs(result.left_rear), std::fabs(result.right_rear)});
    if (peak > max_wheel_speed && peak > 0.0f) {
        const fp32 scale = max_wheel_speed / peak;
        result.left_front *= scale; result.right_front *= scale;
        result.left_rear *= scale; result.right_rear *= scale;
    }
    return result;
}

} // namespace roboctrl::utils::kinematics
