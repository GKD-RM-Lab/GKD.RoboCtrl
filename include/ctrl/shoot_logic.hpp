#pragma once

#include <cmath>

#include "utils/utils.hpp"

namespace roboctrl::ctrl {

inline bool trigger_jammed(
    fp32 feedback_current,
    fp32 feedback_rpm,
    fp32 current_threshold,
    fp32 speed_threshold)
{
    return feedback_current > current_threshold &&
        std::fabs(feedback_rpm) < speed_threshold;
}

inline bool trigger_feed_allowed(
    bool firing,
    bool friction_enabled,
    bool friction_ready,
    bool motors_online,
    bool jam_hold_active)
{
    return firing && friction_enabled && friction_ready && motors_online && !jam_hold_active;
}

} // namespace roboctrl::ctrl
