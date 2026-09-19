#include "ctrl/power_manager.h"
#include "device/chassis/base.hpp"
#include "device/motor/base.hpp"
#include "device/motor/protocol.hpp"
#include "utils/kinematics/mecanum.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace {
using namespace roboctrl;
using namespace roboctrl::ctrl;
using namespace roboctrl::device;

void require_power(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(std::string{"power test: "} + message);
}

power_manager::info_type test_config() {
    power_manager::info_type info;
    info.configured_power_limit = info.offline_power_limit = 80.0f;
    info.torque_per_current = 0.01f;
    info.speed_loss = 0.0f;
    info.torque_loss = 1.0f;
    info.constant_loss = 0.0f;
    info.energy_kp = info.energy_kd = 0.0f;
    return info;
}

std::array<power_wheel, 4> test_wheels() {
    std::array<power_wheel, 4> result;
    for (auto& wheel : result) wheel = {1000.0f, 1000.0f, 0.0f, 0.0f, 20000.0f, true};
    return result;
}

// This fake uses the same final gate as dji_motor::current() and m9025::current().
// It avoids constructing CAN while exercising policy -> bound actuator -> gate.
class current_motor final : public motor_base {
public:
    current_motor() : motor_base{std::chrono::nanoseconds{0}, 1.0f} {}
    awaitable<void> set(float value) override { target = value; co_return; }
    awaitable<void> enable() override { enabled = true; co_return; }
    void disable() override { enabled = false; command = 0.0f; }
    bool supports_current_control() const override { return true; }
    float requested_current() const override { return command; }
    float current_feedback_raw() const override { return command; }
    float max_current() const override { return 20000.0f; }
    float target_angle_speed() const override { return target; }
    void set_output_scale(float value) override { scale = value; }
    void set_current_limit(float value) override { limit = value; }
    std::int16_t wire_current() const {
        return motor_protocol::gated_current(command, scale, limit, enabled, !offline());
    }
    float command {1000.0f}, target {}, scale {1.0f};
    float limit {std::numeric_limits<float>::infinity()};
    bool enabled {true};
};

class power_chassis final : public chassis_base {
public:
    void set_planar_velocity(vectorf) override {}
    void set_planar_x(float) override {}
    void set_planar_y(float) override {}
    vectorf velocity() const override { return {}; }
    void set_rotate_speed(float) override {}
    float rotate_speed() const override { return 0.0f; }
    void set_enabled(bool) override {}
    std::array<motor_base*, 4> wheel_motors() const override {
        return {const_cast<current_motor*>(&motors[0]), const_cast<current_motor*>(&motors[1]),
                const_cast<current_motor*>(&motors[2]), const_cast<current_motor*>(&motors[3])};
    }
    std::array<current_motor, 4> motors;
};

void test_power_allocation() {
    power_manager manager;
    const auto info = test_config();
    require_power(manager.init(info), "valid configuration rejected");
    const auto wheels = test_wheels();
    auto result = manager.allocate(wheels, {}, true, 0.002f);
    require_power(result.limited && result.telemetry_offline, "offline conservative cap not active");
    require_power(result.allocated_power <= 80.0001f, "model power exceeds cap");
    for (float cap : result.current_limits)
        require_power(std::abs(cap - std::sqrt(20.0f) / 0.01f) < 0.001f, "symmetric wheel allocation");

    power_feedback feedback;
    feedback.referee_online = true;
    feedback.referee_power_limit = 20.0f;
    result = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(result.power_limit == 20.0f && result.allocated_power <= 20.0001f,
                  "referee limit not obeyed");
    feedback.cap_online = true;
    feedback.cap_power_limit = 200.0f;
    feedback.cap_energy = 255.0f;
    result = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(result.power_limit == 20.0f, "cap echo overrides referee");
    feedback.referee_power_limit = 0.0f;
    result = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(result.power_limit == 0.0f && result.current_limits[0] == 0.0f,
                  "fresh zero referee budget replaced with offline allowance");
    result = manager.allocate(wheels, feedback, false, 0.002f);
    require_power(std::all_of(result.current_limits.begin(), result.current_limits.end(),
                             [](float value) { return value == 0.0f; }), "NoForce not zero");
    for (float invalid_dt : {0.0f, -1.0f, 2.0f, std::numeric_limits<float>::quiet_NaN()}) {
        result = manager.allocate(wheels, feedback, true, invalid_dt);
        require_power(result.current_limits[0] == 0.0f, "bad dt not zero");
    }
    auto invalid_wheels = wheels;
    invalid_wheels[2].online = false;
    result = manager.allocate(invalid_wheels, {}, true, 0.002f);
    require_power(result.motor_fault && result.current_limits[0] == 0.0f, "one offline wheel not gated");
    invalid_wheels = wheels;
    invalid_wheels[0].requested_current = std::numeric_limits<float>::infinity();
    result = manager.allocate(invalid_wheels, {}, true, 0.002f);
    require_power(result.motor_fault && result.current_limits[0] == 0.0f, "nonfinite input not gated");

    auto friction = info;
    friction.speed_loss = 5.0f;
    require_power(manager.init(friction), "friction config");
    auto coasting = wheels;
    for (auto& wheel : coasting) wheel.angular_speed = 10.0f;
    result = manager.allocate(coasting, {}, true, 0.002f);
    require_power(result.budget_unachievable && result.current_limits[0] == 0.0f &&
                  result.allocated_power == 200.0f, "coasting loss mislabeled as achievable");
}

void test_applied_actuator_caps() {
    power_manager manager;
    require_power(manager.init(test_config()), "integration config");
    power_chassis chassis;
    manager.update(chassis, {}, true, 0.002f);
    double power = 0.0;
    for (auto& motor : chassis.motors) {
        require_power(motor.wire_current() > 0 && motor.wire_current() < 1000, "cap not applied to output");
        const auto first = motor.wire_current();
        motor.command = 20000.0f; // PID changes between policy tick and group send.
        require_power(motor.wire_current() == first, "later PID bypasses absolute allocation");
        motor.command = -20000.0f;
        require_power(motor.wire_current() == -first, "sign change bypasses envelope");
        power += std::pow(motor.wire_current() * 0.01, 2);
    }
    require_power(power <= 80.0, "wire current exceeds allocated budget");
    manager.update(chassis, {}, false, 0.002f);
    for (const auto& motor : chassis.motors) require_power(motor.wire_current() == 0, "NoForce cap missing");
    manager.update(chassis, {}, true, 0.002f);
    chassis.motors[0].disable();
    require_power(chassis.motors[0].wire_current() == 0, "driver disable gate missing");
}

void test_energy_and_rls() {
    power_manager manager;
    auto info = test_config();
    info.energy_kp = 5.0f;
    require_power(manager.init(info), "energy config");
    power_feedback feedback;
    feedback.cap_online = true;
    feedback.cap_power_limit = 50.0f;
    feedback.cap_energy = 0.0f;
    auto wheels = test_wheels();
    auto empty = manager.allocate(wheels, feedback, true, 0.002f);
    feedback.cap_energy = 255.0f;
    auto full = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(empty.power_limit == 0.0f && full.power_limit == 50.0f, "cap energy loop");
    feedback.cap_online = false;
    auto offline = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(offline.power_limit == 80.0f && offline.telemetry_offline, "stale cap fallback");
    feedback.referee_online = feedback.buffer_online = true;
    feedback.referee_power_limit = 50.0f;
    feedback.buffer_energy = 0.0f;
    auto buffer_empty = manager.allocate(wheels, feedback, true, 0.002f);
    feedback.buffer_energy = 60.0f;
    auto buffer_full = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(buffer_empty.power_limit < buffer_full.power_limit, "referee buffer fallback");

    info = test_config();
    info.rls_enabled = true;
    info.rls_initial_covariance = 0.1f;
    info.rls_period = 0.001f;
    require_power(manager.init(info), "RLS config");
    feedback = {};
    feedback.cap_online = true;
    feedback.cap_power_limit = 80.0f;
    feedback.cap_energy = 255.0f;
    feedback.measured_power = 410.0f;
    feedback.measurement_sequence = 1;
    auto first = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(first.rls_updates == 1 && first.torque_loss > 1.0f, "RLS did not use physical loss");
    auto duplicate = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(duplicate.rls_updates == 1, "same measured frame reused");
    feedback.measurement_sequence = 2;
    feedback.cap_online = false;
    auto stale = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(stale.rls_updates == 1, "RLS updated from stale cap");
    feedback.cap_online = true;
    feedback.measured_power = -1.0f;
    auto negative = manager.allocate(wheels, feedback, true, 0.002f);
    require_power(negative.rls_updates == 1, "negative measurement trained loss");
}

void test_model_conversion_and_random_bounds() {
    const power_manager::info_type legacy;
    constexpr double ratio = 3591.0 / 187.0;
    const double current = 14000.0, output_speed = 12.0;
    const double rotor_torque = current * 0.3 / ratio * 20.0 / 16384.0;
    const double rotor_speed = output_speed * ratio;
    const double before = rotor_torque * rotor_speed + 0.22 * rotor_speed +
                          1.2 * rotor_torque * rotor_torque + 2.78;
    const double output_torque = current * legacy.torque_per_current;
    const double after = output_torque * output_speed + legacy.speed_loss * output_speed +
                         legacy.torque_loss * output_torque * output_torque + legacy.constant_loss;
    require_power(std::abs(before - after) < 0.0001, "rotor to output model conversion");

    power_manager manager;
    auto info = test_config();
    info.speed_loss = 0.01f;
    info.constant_loss = 2.0f;
    require_power(manager.init(info), "random config");
    std::mt19937 rng{7301};
    std::uniform_real_distribution<float> current_distribution(-20000.0f, 20000.0f);
    std::uniform_real_distribution<float> speed(-50.0f, 50.0f);
    for (unsigned sample = 0; sample < 2000; ++sample) {
        auto wheels = test_wheels();
        for (auto& wheel : wheels) {
            wheel.requested_current = current_distribution(rng);
            wheel.angular_speed = speed(rng);
            wheel.target_angular_speed = speed(rng);
        }
        const auto output = manager.allocate(wheels, {}, true, 0.002f);
        double bound = info.constant_loss;
        for (std::size_t i = 0; i < wheels.size(); ++i) {
            const auto cap = output.current_limits[i];
            require_power(cap >= 0.0f && cap <= std::abs(wheels[i].requested_current), "allocation amplifies current");
            const double torque = cap * info.torque_per_current;
            bound += torque * std::abs(wheels[i].angular_speed) + info.torque_loss * torque * torque +
                     info.speed_loss * std::abs(wheels[i].angular_speed);
        }
        require_power(bound <= 80.0001, "random envelope exceeds configured limit");
    }
    auto invalid = info;
    invalid.rls_forgetting_factor = std::numeric_limits<float>::quiet_NaN();
    require_power(!power_manager::valid_configuration(invalid), "NaN config accepted");
    invalid = info;
    invalid.offline_power_limit = info.configured_power_limit + 1.0f;
    require_power(!power_manager::valid_configuration(invalid), "offline limit can exceed configured limit");
}

void test_chassis_motor_directions() {
    using utils::kinematics::mecanum_motor_targets;
    constexpr std::array<int, 4> legacy {-1, 1, -1, 1};
    require_power(mecanum_motor_targets({1.0f, 0.0f}, 0.0f, 100.0f, legacy) ==
                  std::array<float, 4>{-1.0f, 1.0f, -1.0f, 1.0f}, "legacy +x signs");
    require_power(mecanum_motor_targets({0.0f, 1.0f}, 0.0f, 100.0f, legacy) ==
                  std::array<float, 4>{1.0f, 1.0f, -1.0f, -1.0f}, "legacy +y signs");
    require_power(mecanum_motor_targets({0.0f, 0.0f}, 1.0f, 100.0f, legacy) ==
                  std::array<float, 4>{1.0f, 1.0f, 1.0f, 1.0f}, "legacy +rotation signs");
    require_power(mecanum_motor_targets({2.0f, 3.0f}, 4.0f, 100.0f, legacy) ==
                  std::array<float, 4>{5.0f, 9.0f, -1.0f, 3.0f}, "rear identity/mixed command");
    const auto limited = mecanum_motor_targets({2.0f, 3.0f}, 4.0f, 2.5f, legacy);
    const std::array<float, 4> expected {5.0f, 9.0f, -1.0f, 3.0f};
    for (std::size_t i = 0; i < limited.size(); ++i)
        require_power(std::abs(limited[i] - expected[i] * (2.5f / 9.0f)) < 1e-6f, "ratio-preserving limit");
    require_power(mecanum_motor_targets({std::numeric_limits<float>::quiet_NaN(), 0.0f},
                  0.0f, 2.5f, legacy) == std::array<float, 4>{}, "NaN chassis command");
}
} // namespace

void test_power_control() {
    test_power_allocation();
    test_applied_actuator_caps();
    test_energy_and_rls();
    test_model_conversion_and_random_bounds();
    test_chassis_motor_directions();
}

#ifdef ROBOCTRL_POWER_CONTROL_TEST_MAIN
int main() { test_power_control(); }
#endif
