#pragma once

#include <cmath>
#include "utils/pid.h"
#include "utils/utils.hpp"

namespace roboctrl::ctrl {

/** Rotate a gimbal-frame command into chassis coordinates (legacy convention). */
inline vectorf gimbal_to_chassis(vectorf velocity, fp32 relative_yaw) {
    const fp32 c = std::cos(relative_yaw), s = std::sin(relative_yaw);
    return {.x = c * velocity.x + s * velocity.y,
            .y = -s * velocity.x + c * velocity.y};
}

/** Follow zero yaw; after spinning, continue its direction until crossing zero. */
class chassis_follow {
public:
    void configure(const utils::rad_pid::params_type& params, fp32 direction,
                   fp32 settle_speed, fp32 tolerance) {
        pid_ = utils::rad_pid{params};
        direction_ = direction;
        settle_speed_ = settle_speed;
        tolerance_ = tolerance;
    }
    fp32 update(fp32 request, fp32 relative_yaw, fp32 dt) {
        const fp32 error = utils::rad_format(relative_yaw);
        if (request != 0.0f) {
            spin_direction_ = std::copysign(1.0f, request);
            returning_ = true;
            last_error_ = error;
            pid_.clean();
            return request;
        }
        if (returning_) {
            // Ignore the +/-pi wrap; accept a real zero crossing between samples.
            const bool crossed = error * last_error_ <= 0.0f &&
                std::abs(error - last_error_) < Pi_f;
            last_error_ = error;
            if (std::abs(error) > tolerance_ && !crossed)
                return spin_direction_ * settle_speed_;
            returning_ = false;
            pid_.clean();
        }
        pid_.update(0.0f, error, dt);
        return direction_ * pid_.state();
    }
    void reset() { returning_ = false; spin_direction_ = last_error_ = 0; pid_.clean(); }
private:
    utils::rad_pid pid_;
    fp32 direction_{1}, settle_speed_{1}, tolerance_{.005f};
    fp32 spin_direction_{}, last_error_{};
    bool returning_{false};
};

} // namespace roboctrl::ctrl
