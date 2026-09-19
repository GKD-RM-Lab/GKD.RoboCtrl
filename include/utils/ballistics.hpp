#pragma once

#include <cstddef>
#include <optional>

namespace roboctrl::utils {

/** Coordinates are x forward, y left, z up; metres, seconds and radians. */
struct ballistic_vector {
    double x {};
    double y {};
    double z {};
};

struct ballistic_target {
    ballistic_vector position {};
    ballistic_vector velocity {};
    double armor_yaw {};
    double yaw_speed {};
    double radius {};
    double alternate_radius {};
    double alternate_height {};
    int armor_count {1};
};

struct ballistic_config {
    double gravity {9.81};
    /** Linear velocity decay coefficient, in 1/s; zero means no drag. */
    double drag {0.0};
    double max_tracking_yaw_speed {5.0};
    double max_flight_time {5.0};
    double position_tolerance {0.001};
    std::size_t bracket_steps {512};
    std::size_t max_iterations {64};
};

enum class ballistic_status {
    success,
    invalid_config,
    invalid_input,
    no_intercept,
    not_converged,
};

struct ballistic_solution {
    ballistic_status status {ballistic_status::invalid_input};
    double yaw {};
    /** Positive above the x-y plane. Legacy BulletSolver::getPitch() negated it. */
    double pitch {};
    double flight_time {};
    ballistic_vector target_position {};
    double residual {};
    int selected_armor {};
    bool tracking_rotation {};
    std::size_t iterations {};
    explicit operator bool() const { return status == ballistic_status::success; }
};

/** Legacy uncalibrated coefficient lookup, made opt-in instead of implicit. */
std::optional<double> legacy_ballistic_drag(double bullet_speed);

/** Evaluate the same linear-drag trajectory used by solve_ballistics(). */
std::optional<ballistic_vector> ballistic_position(
    const ballistic_config& config, double speed, double yaw, double pitch, double time);

/**
 * Bounded search for the first bracketed positive-time intercept in max_flight_time.
 * Retains the old adjacent-armor selection and high-spin front-surface prediction.
 * no_intercept means no root was bracketed in this horizon, not a reachability proof.
 * Pure calculation: it neither commands hardware nor authorizes firing.
 */
ballistic_solution solve_ballistics(
    const ballistic_config& config, const ballistic_target& target, double bullet_speed);

} // namespace roboctrl::utils
