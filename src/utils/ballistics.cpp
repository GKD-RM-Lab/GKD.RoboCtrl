/*
 * Target prediction and linear-drag model adapted from GKD_Control BulletSolver.
 * Copyright (c) 2021, Qiayuan Liao. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils/ballistics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace roboctrl::utils {
namespace {

constexpr double pi = std::numbers::pi_v<double>;

bool finite(ballistic_vector v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool valid(const ballistic_config& c) {
    return std::isfinite(c.gravity) && c.gravity >= 0.0 &&
           std::isfinite(c.drag) && c.drag >= 0.0 &&
           std::isfinite(c.max_tracking_yaw_speed) && c.max_tracking_yaw_speed > 0.0 &&
           std::isfinite(c.max_flight_time) && c.max_flight_time > 0.0 &&
           std::isfinite(c.position_tolerance) && c.position_tolerance > 0.0 &&
           c.bracket_steps >= 2 && c.bracket_steps <= 65536 &&
           c.max_iterations > 0 && c.max_iterations <= 256;
}

struct flight_terms {
    double displacement;
    double drop;
};

flight_terms terms(const ballistic_config& c, double time) {
    const double kt = c.drag * time;
    // The series avoids subtracting two nearly equal values in (t - A) / k.
    if (std::abs(kt) < 1e-4) {
        const double a = time * (1.0 + kt * (-0.5 + kt * (1.0 / 6.0 - kt / 24.0)));
        const double drop = c.gravity * time * time *
                            (0.5 + kt * (-1.0 / 6.0 + kt * (1.0 / 24.0 - kt / 120.0)));
        return {a, drop};
    }
    const double a = -std::expm1(-kt) / c.drag;
    return {a, c.gravity * (time - a) / c.drag};
}

ballistic_vector predict(const ballistic_target& target, double time,
                         int selected, bool tracking) {
    ballistic_vector result {
        target.position.x + target.velocity.x * time,
        target.position.y + target.velocity.y * time,
        target.position.z + target.velocity.z * time};
    const bool alternate = target.armor_count == 4 && selected != 0;
    const double radius = alternate ? target.alternate_radius : target.radius;
    const double angle = tracking
        ? target.armor_yaw + target.yaw_speed * time + selected * 2.0 * pi / target.armor_count
        : std::atan2(result.y, result.x);
    result.x -= radius * std::cos(angle);
    result.y -= radius * std::sin(angle);
    result.z += alternate ? target.alternate_height : 0.0;
    return result;
}

} // namespace

std::optional<double> legacy_ballistic_drag(double bullet_speed) {
    if (!std::isfinite(bullet_speed) || bullet_speed <= 0.0) return std::nullopt;
    if (bullet_speed < 12.5) return 0.45;
    if (bullet_speed < 15.5) return 1.0;
    if (bullet_speed < 17.0) return 0.7;
    if (bullet_speed < 24.0) return 0.55;
    return 5.0;
}

std::optional<ballistic_vector> ballistic_position(
    const ballistic_config& config, double speed, double yaw, double pitch, double time)
{
    if (!valid(config) || !std::isfinite(speed) || speed <= 0.0 ||
        !std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(time) || time < 0.0)
        return std::nullopt;
    const auto f = terms(config, time);
    const double horizontal = speed * std::cos(pitch) * f.displacement;
    ballistic_vector result {horizontal * std::cos(yaw), horizontal * std::sin(yaw),
                             speed * std::sin(pitch) * f.displacement - f.drop};
    return finite(result) ? std::optional{result} : std::nullopt;
}

ballistic_solution solve_ballistics(
    const ballistic_config& config, const ballistic_target& target, double bullet_speed)
{
    ballistic_solution result;
    if (!valid(config)) {
        result.status = ballistic_status::invalid_config;
        return result;
    }
    const double range = std::hypot(target.position.x, target.position.y);
    if (!finite(target.position) || !finite(target.velocity) ||
        !std::isfinite(bullet_speed) || bullet_speed <= 0.0 ||
        !std::isfinite(target.armor_yaw) || !std::isfinite(target.yaw_speed) ||
        !std::isfinite(target.radius) || target.radius < 0.0 ||
        !std::isfinite(target.alternate_radius) || target.alternate_radius < 0.0 ||
        !std::isfinite(target.alternate_height) || target.armor_count < 1 || target.armor_count > 16 ||
        range <= std::max(target.radius, target.alternate_radius)) return result;

    result.tracking_rotation = std::abs(target.yaw_speed) < config.max_tracking_yaw_speed;
    const double bearing = std::atan2(target.position.y, target.position.x);
    double rough_time = std::hypot(range, target.position.z) / bullet_speed;
    const double drag_fraction = config.drag * rough_time;
    if (config.drag > 0.0 && drag_fraction < 1.0)
        rough_time = -std::log1p(-drag_fraction) / config.drag;
    const double visible_angle = std::acos(std::clamp(target.radius / range, 0.0, 1.0));
    const double switch_angle = result.tracking_rotation
        ? visible_angle - pi / 12.0 + (-visible_angle + pi / 6.0) *
          std::abs(target.yaw_speed) / config.max_tracking_yaw_speed
        : pi / 12.0;
    const double angle_error = std::remainder(
        target.armor_yaw + target.yaw_speed * rough_time - bearing, 2.0 * pi);
    if (target.armor_count > 1 &&
        ((angle_error > switch_angle && target.yaw_speed > 0.0) ||
         (angle_error < -switch_angle && target.yaw_speed < 0.0)))
        result.selected_armor = target.yaw_speed > 0.0 ? -1 : 1;
    auto error = [&](double time) {
        const auto p = predict(target, time, result.selected_armor, result.tracking_rotation);
        const auto f = terms(config, time);
        return std::hypot(std::hypot(p.x, p.y), p.z + f.drop) - bullet_speed * f.displacement;
    };
    double left = 0.0;
    double right = 0.0;
    bool bracketed = false;
    for (std::size_t i = 1; i <= config.bracket_steps; ++i) {
        right = config.max_flight_time * (static_cast<double>(i) / config.bracket_steps);
        const double e = error(right);
        if (!std::isfinite(e)) {
            result.status = ballistic_status::invalid_input;
            return result;
        }
        if (e <= 0.0) { bracketed = true; break; }
        left = right;
    }
    if (!bracketed) {
        result.status = ballistic_status::no_intercept;
        return result;
    }
    for (std::size_t i = 0; i < config.max_iterations; ++i) {
        const double time = 0.5 * (left + right);
        const double e = error(time);
        result.iterations = i + 1;
        if (std::abs(e) <= config.position_tolerance) {
            const auto p = predict(target, time, result.selected_armor, result.tracking_rotation);
            const auto f = terms(config, time);
            result.yaw = std::atan2(p.y, p.x);
            result.pitch = std::atan2(p.z + f.drop, std::hypot(p.x, p.y));
            result.flight_time = time;
            result.target_position = p;
            result.residual = std::abs(e);
            result.status = ballistic_status::success;
            return result;
        }
        if (e > 0.0) left = time; else right = time;
    }
    result.status = ballistic_status::not_converged;
    return result;
}

} // namespace roboctrl::utils
