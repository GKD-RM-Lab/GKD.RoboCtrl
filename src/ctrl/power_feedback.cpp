#include "ctrl/power_manager.h"

#include <algorithm>
#include <cmath>

#include "device/referee/referee.h"
#include "device/super_cap.h"

namespace roboctrl::ctrl {

awaitable<void> power_manager::update(device::chassis_base& chassis,
                                      bool output_allowed, float dt_seconds) {
    power_feedback feedback;
    auto& cap = roboctrl::get<device::super_cap>();
    if (cap.configured() && !cap.offline() && cap.error_code() == 0) {
        feedback.cap_online = true;
        feedback.cap_power_limit = cap.chassis_power_limit();
        feedback.cap_energy = cap.energy();
        feedback.measured_power = cap.chassis_power();
        feedback.measurement_sequence = cap.sample_sequence();
    }
    auto& referee = roboctrl::get<device::referee>();
    if (referee.configured() && !referee.offline()) {
        const auto now = device::referee_protocol::clock::now();
        const auto& data = referee.data();
        feedback.referee_online = data.robot.fresh(now, referee.timeout());
        if (feedback.referee_online) {
            feedback.referee_power_limit = data.robot.value.chassis_power_limit;
            output_allowed = output_allowed && data.robot.value.chassis_power;
        }
        feedback.buffer_online = data.power.fresh(now, referee.timeout());
        if (feedback.buffer_online) feedback.buffer_energy = data.power.value.buffer_energy;
    }
    update(chassis, feedback, output_allowed, dt_seconds);
    if (cap.configured()) {
        // The cap's input limit follows the referee limit; chassis boost is a
        // separate discharge budget and must never raise the charging limit.
        const float source_limit = feedback.referee_online && feedback.referee_power_limit >= 0.0f
            ? feedback.referee_power_limit : info_.offline_power_limit;
        const auto limit = static_cast<std::uint16_t>(std::clamp(source_limit, 0.0f, 65535.0f));
        co_await cap.set(enabled() && output_allowed && !status_.motor_fault, limit);
    }
}

} // namespace roboctrl::ctrl
