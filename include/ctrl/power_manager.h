#pragma once

#include <array>
#include <cstdint>

#include "core/async.hpp"
#include "utils/singleton.hpp"

namespace roboctrl::device { class chassis_base; }

namespace roboctrl::ctrl {

/** Snapshot in output-shaft rad/s and the motor's raw current command units. */
struct power_wheel {
    float requested_current {};
    float measured_current {};
    float angular_speed {};
    float target_angular_speed {};
    float max_current {};
    bool online {false};
};

/** Validity is supplied by the device freshness checks, never by payload presence. */
struct power_feedback {
    bool referee_online {false};
    float referee_power_limit {};
    bool buffer_online {false};
    float buffer_energy {};
    bool cap_online {false};
    float cap_power_limit {};
    float cap_energy {}; // Legacy capacitor protocol index, 0..255; not joules.
    float measured_power {}; // Capacitor input, W; no guessed referee field.
    std::uint64_t measurement_sequence {};
};

struct power_allocation {
    std::array<float, 4> current_limits {};
    float power_limit {};
    float requested_power {};
    float allocated_power {};
    float measured_power {};
    float speed_loss {};
    float torque_loss {};
    bool limited {false};
    bool motor_fault {false};
    bool telemetry_offline {true};
    bool budget_unachievable {false};
    std::uint64_t rls_updates {};
};

/** Control policy only: bounded work on each motion-control tick, no extra thread. */
class power_manager : public utils::singleton_base<power_manager> {
public:
    struct info_type {
        using owner_type = power_manager;
        bool enabled {true};
        float configured_power_limit {35.0f};
        float offline_power_limit {35.0f};
        float max_cap_boost {0.0f};
        // Algebraic conversion of the legacy M3508 rotor model to output shaft.
        float torque_per_current {0.3f * 20.0f / 16384.0f};
        float speed_loss {0.22f * (3591.0f / 187.0f)};
        float torque_loss {1.2f * (187.0f / 3591.0f) * (187.0f / 3591.0f)};
        float constant_loss {2.78f};
        float error_weight_low {1.0f}; // rad/s, not the legacy mixed-unit error.
        float error_weight_high {2.0f};
        float cap_base_energy {100.0f};
        float cap_full_energy {250.0f};
        float buffer_base_energy {50.0f};
        float buffer_full_energy {60.0f};
        float energy_kp {50.0f};
        float energy_kd {0.0002f}; // Legacy kd=0.2 per 1 ms -> seconds derivative.
        bool rls_enabled {false};
        float rls_forgetting_factor {0.9999f};
        float rls_initial_covariance {0.00001f};
        float rls_period {0.01f}; // Seconds; consume a measurement only once.
        float max_speed_loss {20.0f};
        float max_torque_loss {20.0f};
    };

    power_manager() = default;
    static bool valid_configuration(const info_type& info);
    bool init(const info_type& info);
    bool configured() const { return configured_; }
    bool enabled() const { return configured_ && info_.enabled; }
    const power_allocation& status() const { return status_; }

    /** Pure numeric entry point; no hardware access or asynchronous work. */
    power_allocation allocate(const std::array<power_wheel, 4>& wheels,
                              const power_feedback& feedback, bool output_allowed,
                              float dt_seconds);
    /** Apply the exact allocated absolute caps to the bound wheel actuators. */
    void update(device::chassis_base& chassis, const power_feedback& feedback,
                bool output_allowed, float dt_seconds);
    /** Capture fresh referee/capacitor telemetry in the control layer. */
    awaitable<void> update(device::chassis_base& chassis, bool output_allowed, float dt_seconds);

private:
    float energy_limit(const power_feedback& feedback, float dt_seconds);
    void update_model(const std::array<power_wheel, 4>& wheels,
                      const power_feedback& feedback, float dt_seconds);
    void reset_energy();

    info_type info_ {};
    bool configured_ {false};
    power_allocation status_ {};
    float speed_loss_ {};
    float torque_loss_ {};
    std::array<std::array<double, 2>, 2> covariance_ {};
    float previous_base_error_ {};
    float previous_full_error_ {};
    unsigned energy_source_ {};
    float rls_elapsed_ {};
    std::uint64_t last_measurement_sequence_ {};
    std::uint64_t rls_updates_ {};
};

static_assert(utils::singleton<power_manager>);

} // namespace roboctrl::ctrl
