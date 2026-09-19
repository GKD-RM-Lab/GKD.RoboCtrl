#include "utils/ballistics.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using namespace roboctrl::utils;

void require(bool condition) {
    if (!condition) throw std::runtime_error("ballistics test failed");
}

double distance(ballistic_vector a, ballistic_vector b) {
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

void verify_impact(const ballistic_config& config, const ballistic_solution& solution, double speed) {
    require(static_cast<bool>(solution));
    auto impact = ballistic_position(config, speed, solution.yaw, solution.pitch, solution.flight_time);
    require(impact.has_value());
    require(distance(*impact, solution.target_position) <= config.position_tolerance * 1.01);
}

} // namespace

void run_ballistics_tests() {
    ballistic_config config {.position_tolerance = 1e-8};
    ballistic_target target {.position = {10.0, 0.0, 0.0}};
    auto solution = solve_ballistics(config, target, 20.0);
    verify_impact(config, solution, 20.0);
    const double analytic_pitch = 0.5 * std::asin(config.gravity * 10.0 / (20.0 * 20.0));
    require(std::abs(solution.pitch - analytic_pitch) < 1e-8);
    require(std::abs(solution.flight_time - 10.0 / (20.0 * std::cos(analytic_pitch))) < 1e-8);
    require(solution.yaw == 0.0);

    // Independent closed trajectory: k=0, v=20, pitch=30deg, t=0.5s.
    auto point = ballistic_position(config, 20.0, 0.0, std::asin(0.5), 0.5);
    require(point && std::abs(point->x - 5.0 * std::sqrt(3.0)) < 1e-12);
    require(std::abs(point->z - (5.0 - 0.5 * 9.81 * 0.25)) < 1e-12);
    config.drag = 0.4;
    point = ballistic_position(config, 20.0, 0.0, std::asin(0.5), 0.5);
    const double a = (1.0 - std::exp(-0.2)) / 0.4;
    require(point && std::abs(point->x - 20.0 * std::sqrt(0.75) * a) < 1e-12);
    require(std::abs(point->z - ((10.0 + 9.81 / 0.4) * a - 9.81 * 0.5 / 0.4)) < 1e-12);
    target.position = *point;
    solution = solve_ballistics(config, target, 20.0);
    verify_impact(config, solution, 20.0);
    require(std::abs(solution.pitch - std::asin(0.5)) < 1e-8);
    require(std::abs(solution.flight_time - 0.5) < 1e-8);

    // k -> 0 is continuous; zero is no longer silently replaced by 0.001.
    config.drag = 1e-12;
    point = ballistic_position(config, 20.0, 0.0, std::asin(0.5), 0.5);
    require(point && std::abs(point->z - (5.0 - 0.5 * 9.81 * 0.25)) < 1e-10);

    config.drag = 0.0;
    config.gravity = 0.0;
    target = {.position = {10.0, 0.0, 0.0}, .velocity = {2.0, 0.0, 0.0}};
    solution = solve_ballistics(config, target, 20.0);
    verify_impact(config, solution, 20.0);
    require(std::abs(solution.flight_time - 10.0 / 18.0) < 1e-8);
    require(std::abs(solution.target_position.x - (10.0 + 2.0 * solution.flight_time)) < 1e-12);

    config.gravity = 9.81;
    target = {.position = {8.0, 1.0, 0.2}, .velocity = {0.3, -0.4, 0.1},
              .armor_yaw = 1.0, .yaw_speed = 1.2, .radius = 0.3,
              .alternate_radius = 0.2, .alternate_height = 0.1, .armor_count = 4};
    solution = solve_ballistics(config, target, 25.0);
    verify_impact(config, solution, 25.0);
    require(solution.tracking_rotation);
    const double t = solution.flight_time;
    const bool alternate = solution.selected_armor != 0;
    const double r = alternate ? target.alternate_radius : target.radius;
    const double angle = target.armor_yaw + target.yaw_speed * t +
                         solution.selected_armor * 2.0 * std::acos(-1.0) / target.armor_count;
    require(std::abs(solution.target_position.x - (8.0 + 0.3 * t - r * std::cos(angle))) < 1e-12);
    require(std::abs(solution.target_position.y - (1.0 - 0.4 * t - r * std::sin(angle))) < 1e-12);
    require(std::abs(solution.target_position.z - (0.2 + 0.1 * t + (alternate ? 0.1 : 0.0))) < 1e-12);
    target.yaw_speed = 6.0;
    solution = solve_ballistics(config, target, 25.0);
    verify_impact(config, solution, 25.0);
    require(!solution.tracking_rotation);

    target = {.position = {100.0, 0.0, 0.0}};
    require(solve_ballistics(config, target, 5.0).status == ballistic_status::no_intercept);
    target.position = {10.0, 0.0, 0.0};
    target.velocity.x = 25.0;
    require(solve_ballistics(config, target, 20.0).status == ballistic_status::no_intercept);
    target.velocity = {};
    require(solve_ballistics(config, target, 0.0).status == ballistic_status::invalid_input);
    require(solve_ballistics(config, target, std::numeric_limits<double>::infinity()).status ==
            ballistic_status::invalid_input);
    target.position.z = std::numeric_limits<double>::quiet_NaN();
    require(solve_ballistics(config, target, 20.0).status == ballistic_status::invalid_input);
    target = {.position = {10.0, 0.0, 0.0}, .armor_count = 0};
    require(solve_ballistics(config, target, 20.0).status == ballistic_status::invalid_input);
    target.armor_count = 1;
    config.drag = -1.0;
    require(solve_ballistics(config, target, 20.0).status == ballistic_status::invalid_config);
    config.drag = 0.0;
    config.max_iterations = 1;
    require(solve_ballistics(config, target, 20.0).status == ballistic_status::not_converged);
    require(!ballistic_position(config, 20.0, 0.0, 0.0, -1.0));
    require(*legacy_ballistic_drag(10.0) == 0.45 && *legacy_ballistic_drag(15.0) == 1.0);
    require(*legacy_ballistic_drag(16.0) == 0.7 && *legacy_ballistic_drag(18.0) == 0.55);
    require(*legacy_ballistic_drag(30.0) == 5.0 && !legacy_ballistic_drag(0.0));
}

#ifdef ROBOCTRL_BALLISTICS_TEST_MAIN
int main() { run_ballistics_tests(); }
#endif
