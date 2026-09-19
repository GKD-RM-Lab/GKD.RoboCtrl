#pragma once
#include <cmath>
#include "device/imu/base.hpp"

namespace roboctrl::device {
/** Body gyro projected onto the yaw axis, as used by the source controller. */
inline fp32 compensated_yaw_rate(fp32 pitch, three_axis gyro) {
    return std::cos(pitch) * gyro.z - std::sin(pitch) * gyro.x;
}

/** Continuous enabled, online tolerance dwell for mechanical centering. */
class gimbal_settle {
public:
    bool update(bool enabled, bool online, fp32 yaw_error, fp32 pitch_error,
                fp32 tolerance, fp32 settle_seconds, fp32 dt) {
        const bool within = enabled && online && std::isfinite(yaw_error) &&
            std::isfinite(pitch_error) && std::abs(yaw_error) <= tolerance &&
            std::abs(pitch_error) <= tolerance;
        if (!within) elapsed_ = 0;
        else if (dt > 0) elapsed_ += dt;
        return within && elapsed_ >= settle_seconds;
    }
    void reset() { elapsed_ = 0; }
private:
    fp32 elapsed_{};
};
}
