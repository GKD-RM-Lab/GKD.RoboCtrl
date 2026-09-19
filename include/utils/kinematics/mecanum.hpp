#pragma once

#include <algorithm>
#include <array>
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

/** Final output-axis motor targets, LF/RF/LR/RR, in m/s. */
inline std::array<fp32, 4> mecanum_motor_targets(
    vectorf velocity, fp32 rotate_speed, fp32 max_wheel_speed,
    const std::array<int, 4>& directions)
{
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) ||
        !std::isfinite(rotate_speed) || !std::isfinite(max_wheel_speed) || max_wheel_speed <= 0.0f ||
        !std::all_of(directions.begin(), directions.end(), [](int value) { return value == -1 || value == 1; }))
        return {};
    const auto wheels = inverse_mecanum(velocity, rotate_speed, max_wheel_speed);
    return {wheels.left_front * directions[0], wheels.right_front * directions[1],
            wheels.left_rear * directions[2], wheels.right_rear * directions[3]};
}

} // namespace roboctrl::utils::kinematics
