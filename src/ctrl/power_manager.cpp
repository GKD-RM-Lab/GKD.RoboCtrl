#include "ctrl/power_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "device/chassis/base.hpp"
#include "device/motor/base.hpp"

namespace roboctrl::ctrl {
namespace {

bool nonnegative(float value) { return std::isfinite(value) && value >= 0.0f; }
bool positive(float value) { return std::isfinite(value) && value > 0.0f; }

bool valid_wheel(const power_wheel& wheel) {
    return wheel.online && std::isfinite(wheel.requested_current) &&
           std::isfinite(wheel.measured_current) && std::isfinite(wheel.angular_speed) &&
           std::isfinite(wheel.target_angular_speed) && positive(wheel.max_current);
}

} // namespace

bool power_manager::valid_configuration(const info_type& info) {
    return positive(info.configured_power_limit) && positive(info.offline_power_limit) &&
           info.offline_power_limit <= info.configured_power_limit &&
           nonnegative(info.max_cap_boost) && positive(info.torque_per_current) &&
           nonnegative(info.speed_loss) && nonnegative(info.torque_loss) &&
           nonnegative(info.constant_loss) && nonnegative(info.error_weight_low) &&
           positive(info.error_weight_high) && info.error_weight_high > info.error_weight_low &&
           nonnegative(info.cap_base_energy) && positive(info.cap_full_energy) &&
           info.cap_base_energy <= info.cap_full_energy && info.cap_full_energy <= 255.0f &&
           nonnegative(info.buffer_base_energy) && positive(info.buffer_full_energy) &&
           info.buffer_base_energy <= info.buffer_full_energy &&
           nonnegative(info.energy_kp) && nonnegative(info.energy_kd) &&
           positive(info.rls_forgetting_factor) && info.rls_forgetting_factor <= 1.0f &&
           positive(info.rls_initial_covariance) && positive(info.rls_period) &&
           positive(info.max_speed_loss) && positive(info.max_torque_loss) &&
           info.speed_loss <= info.max_speed_loss && info.torque_loss <= info.max_torque_loss;
}

bool power_manager::init(const info_type& info) {
    if (!valid_configuration(info)) return false;
    info_ = info;
    configured_ = true;
    speed_loss_ = info.speed_loss;
    torque_loss_ = info.torque_loss;
    covariance_ = {{{info.rls_initial_covariance, 0.0}, {0.0, info.rls_initial_covariance}}};
    rls_elapsed_ = 0.0f;
    last_measurement_sequence_ = 0;
    rls_updates_ = 0;
    status_ = {};
    reset_energy();
    return true;
}

void power_manager::reset_energy() {
    energy_source_ = 0;
    previous_base_error_ = previous_full_error_ = 0.0f;
}

float power_manager::energy_limit(const power_feedback& feedback, float dt_seconds) {
    const bool referee_online = feedback.referee_online && nonnegative(feedback.referee_power_limit);
    const bool cap_online = feedback.cap_online && nonnegative(feedback.cap_power_limit) &&
                            nonnegative(feedback.cap_energy) && feedback.cap_energy <= 255.0f;
    const bool buffer_online = referee_online && feedback.buffer_online &&
                               nonnegative(feedback.buffer_energy);
    status_.telemetry_offline = !referee_online && !cap_online;
    if (status_.telemetry_offline) {
        reset_energy();
        return info_.offline_power_limit;
    }

    // A fresh referee limit always takes precedence over a capacitor echo.
    const float source_limit = referee_online ? feedback.referee_power_limit : feedback.cap_power_limit;
    const float ceiling = std::min(info_.configured_power_limit,
                                  source_limit + (cap_online ? info_.max_cap_boost : 0.0f));
    unsigned source = cap_online ? 1U : buffer_online ? 2U : 0U;
    if (!source) {
        reset_energy();
        return std::min(ceiling, source_limit);
    }
    const float energy = cap_online ? feedback.cap_energy : feedback.buffer_energy;
    const float base_target = cap_online ? info_.cap_base_energy : info_.buffer_base_energy;
    const float full_target = cap_online ? info_.cap_full_energy : info_.buffer_full_energy;
    const float base_error = std::sqrt(base_target) - std::sqrt(energy);
    const float full_error = std::sqrt(full_target) - std::sqrt(energy);
    const bool derivative_valid = energy_source_ == source;
    const float base_pd = info_.energy_kp * base_error +
        (derivative_valid ? info_.energy_kd * (base_error - previous_base_error_) / dt_seconds : 0.0f);
    const float full_pd = info_.energy_kp * full_error +
        (derivative_valid ? info_.energy_kd * (full_error - previous_full_error_) / dt_seconds : 0.0f);
    previous_base_error_ = base_error;
    previous_full_error_ = full_error;
    energy_source_ = source;
    const float base_limit = std::clamp(source_limit - base_pd, 0.0f, ceiling);
    const float full_limit = std::clamp(source_limit - full_pd, 0.0f, ceiling);
    // Preserve the two legacy energy thresholds without forcing a minimum draw
    // when the buffer is empty. The configured limit competes with both bounds.
    return std::clamp(ceiling, std::min(base_limit, full_limit), std::max(base_limit, full_limit));
}

void power_manager::update_model(const std::array<power_wheel, 4>& wheels,
                                 const power_feedback& feedback, float dt_seconds) {
    rls_elapsed_ = std::min(rls_elapsed_ + dt_seconds, info_.rls_period);
    if (!info_.rls_enabled || !feedback.cap_online ||
        !std::isfinite(feedback.measured_power) || feedback.measured_power <= 5.0f ||
        feedback.measurement_sequence == 0 ||
        feedback.measurement_sequence == last_measurement_sequence_ ||
        rls_elapsed_ < info_.rls_period) return;

    last_measurement_sequence_ = feedback.measurement_sequence;
    rls_elapsed_ = 0.0f;
    std::array<double, 2> samples {};
    double mechanical_power = 0.0;
    for (const auto& wheel : wheels) {
        const double torque = wheel.measured_current * info_.torque_per_current;
        mechanical_power += torque * wheel.angular_speed;
        samples[0] += std::abs(wheel.angular_speed);
        samples[1] += torque * torque;
    }
    const double loss = feedback.measured_power - mechanical_power - info_.constant_loss;
    if (!std::isfinite(loss) || loss < 0.0 || samples[0] + samples[1] < 1e-9) return;
    const std::array<double, 2> px {
        covariance_[0][0] * samples[0] + covariance_[0][1] * samples[1],
        covariance_[1][0] * samples[0] + covariance_[1][1] * samples[1]};
    const double denominator = info_.rls_forgetting_factor + samples[0] * px[0] + samples[1] * px[1];
    if (!std::isfinite(denominator) || denominator <= 1e-12) return;
    const double residual = loss - samples[0] * speed_loss_ - samples[1] * torque_loss_;
    const double new_speed_loss = speed_loss_ + px[0] / denominator * residual;
    const double new_torque_loss = torque_loss_ + px[1] / denominator * residual;
    if (!std::isfinite(new_speed_loss) || !std::isfinite(new_torque_loss) ||
        new_speed_loss < 0.0 || new_speed_loss > info_.max_speed_loss ||
        new_torque_loss < 0.0 || new_torque_loss > info_.max_torque_loss) return;

    auto next = covariance_;
    for (std::size_t row = 0; row < 2; ++row) {
        for (std::size_t col = 0; col < 2; ++col) {
            next[row][col] = (covariance_[row][col] - px[row] * px[col] / denominator) /
                             info_.rls_forgetting_factor;
            if (!std::isfinite(next[row][col]) || std::abs(next[row][col]) > 1e12) return;
        }
    }
    if (next[0][0] < 0.0 || next[1][1] < 0.0 ||
        next[0][0] * next[1][1] < next[0][1] * next[1][0] - 1e-10) return;
    covariance_ = next;
    speed_loss_ = static_cast<float>(new_speed_loss);
    torque_loss_ = static_cast<float>(new_torque_loss);
    ++rls_updates_;
}

power_allocation power_manager::allocate(const std::array<power_wheel, 4>& wheels,
                                        const power_feedback& feedback, bool output_allowed,
                                        float dt_seconds) {
    status_ = {};
    status_.rls_updates = rls_updates_;
    status_.speed_loss = speed_loss_;
    status_.torque_loss = torque_loss_;
    if (!output_allowed || !positive(dt_seconds) || dt_seconds > 1.0f) {
        reset_energy();
        return status_;
    }
    status_.motor_fault = !std::all_of(wheels.begin(), wheels.end(), valid_wheel);
    if (status_.motor_fault) {
        reset_energy();
        return status_;
    }
    if (!enabled()) {
        for (std::size_t i = 0; i < wheels.size(); ++i) status_.current_limits[i] = wheels[i].max_current;
        return status_;
    }

    update_model(wheels, feedback, dt_seconds);
    status_.speed_loss = speed_loss_;
    status_.torque_loss = torque_loss_;
    status_.rls_updates = rls_updates_;
    status_.power_limit = energy_limit(feedback, dt_seconds);
    status_.measured_power = feedback.cap_online && std::isfinite(feedback.measured_power)
                           ? feedback.measured_power : 0.0f;
    std::array<double, 4> demand {}, error {}, allocated {};
    double baseline = info_.constant_loss;
    double sum_demand = 0.0, sum_error = 0.0;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const auto& wheel = wheels[i];
        const double current = std::min(std::abs(wheel.requested_current), wheel.max_current);
        const double torque = current * info_.torque_per_current;
        // No regenerative credit: this is an upper envelope valid for either
        // current sign until the next tick, unlike the signed legacy estimate.
        demand[i] = torque * std::abs(wheel.angular_speed) + torque_loss_ * torque * torque;
        baseline += speed_loss_ * std::abs(wheel.angular_speed);
        error[i] = std::abs(wheel.target_angular_speed - wheel.angular_speed);
        sum_demand += demand[i];
        if (demand[i] > 0.0) sum_error += error[i];
    }
    status_.requested_power = static_cast<float>(baseline + sum_demand);
    if (!std::isfinite(status_.requested_power) || !nonnegative(status_.power_limit)) {
        status_.motor_fault = true;
        return status_;
    }
    if (baseline > status_.power_limit) {
        // Zero current cannot remove existing coasting/friction losses.
        status_.budget_unachievable = true;
        status_.limited = true;
        status_.allocated_power = static_cast<float>(baseline);
        return status_;
    }

    double remaining = std::max(0.0, static_cast<double>(status_.power_limit) - baseline);
    const double confidence = std::clamp((sum_error - info_.error_weight_low) /
                                        (info_.error_weight_high - info_.error_weight_low), 0.0, 1.0);
    std::array<double, 4> weights {};
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        weights[i] = (sum_error > 0.0 ? confidence * error[i] / sum_error : 0.0) +
                     (sum_demand > 0.0 ? (1.0 - confidence) * demand[i] / sum_demand : 0.0);
    }
    // Water filling redistributes unused wheel budgets; at most four caps can
    // saturate. A final proportional pass handles zero-error demand safely.
    for (unsigned pass = 0; pass < 5 && remaining > 1e-9; ++pass) {
        double weight_sum = 0.0;
        for (std::size_t i = 0; i < wheels.size(); ++i)
            if (demand[i] > allocated[i] + 1e-9) weight_sum += pass == 4 ? demand[i] : weights[i];
        if (weight_sum <= 0.0) continue;
        const double available = remaining;
        for (std::size_t i = 0; i < wheels.size(); ++i) {
            if (demand[i] <= allocated[i] + 1e-9) continue;
            const double weight = pass == 4 ? demand[i] : weights[i];
            const double grant = std::min(demand[i] - allocated[i], available * weight / weight_sum);
            allocated[i] += grant;
            remaining -= grant;
        }
    }

    double total = baseline;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const double speed = std::abs(wheels[i].angular_speed);
        const double requested = std::min(std::abs(wheels[i].requested_current), wheels[i].max_current);
        double current = requested;
        if (allocated[i] < demand[i]) {
            const double torque = torque_loss_ > 0.0f
                ? 2.0 * allocated[i] / (speed + std::sqrt(speed * speed + 4.0 * torque_loss_ * allocated[i]) + 1e-30)
                : speed > 0.0 ? allocated[i] / speed : 0.0;
            current = std::min(requested, torque / info_.torque_per_current);
        }
        // Round toward zero so converting the cap to float cannot enlarge it.
        status_.current_limits[i] = std::max(0.0f, std::nextafter(static_cast<float>(current), 0.0f));
        const double torque = status_.current_limits[i] * info_.torque_per_current;
        total += torque * speed + torque_loss_ * torque * torque;
    }
    status_.allocated_power = static_cast<float>(total);
    status_.limited = status_.requested_power > status_.power_limit;
    return status_;
}

void power_manager::update(device::chassis_base& chassis, const power_feedback& feedback,
                           bool output_allowed, float dt_seconds) {
    const auto motors = chassis.wheel_motors();
    std::array<power_wheel, 4> wheels {};
    for (std::size_t i = 0; i < motors.size(); ++i) {
        const auto* motor = motors[i];
        if (!motor) continue;
        wheels[i] = {motor->requested_current(), motor->current_feedback_raw(), motor->angle_speed(),
                     motor->target_angle_speed(), motor->max_current(),
                     !motor->offline() && motor->supports_current_control()};
    }
    const auto allocation = allocate(wheels, feedback, output_allowed, dt_seconds);
    for (std::size_t i = 0; i < motors.size(); ++i) {
        if (!motors[i]) continue;
        motors[i]->set_output_scale(1.0f);
        motors[i]->set_current_limit(allocation.current_limits[i]);
    }
}

} // namespace roboctrl::ctrl
