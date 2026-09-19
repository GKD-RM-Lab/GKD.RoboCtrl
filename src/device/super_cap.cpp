#include "device/super_cap.h"
#include "device/super_cap_protocol.hpp"
#include "io/can.h"

using namespace roboctrl;
using namespace roboctrl::device;

bool super_cap::init(const info_type& info) {
    if (!valid_configuration(info)) throw std::invalid_argument("invalid super capacitor configuration");
    if (configured_) throw std::logic_error("super capacitor already configured");
    info_ = info;
    configured_ = true;
    return true;
}

void super_cap::connect() {
    if (!configured_) throw std::logic_error("super capacitor must initialize before connect");
    if (connected_) return;
    get<io::can>(info_.can_name).on_data(info_.receive_id, [this](io::byte_span data) {
        const auto feedback = super_cap_protocol::decode(data);
        if (!feedback) return;
        chassis_power_ = feedback->chassis_power;
        chassis_power_limit_ = feedback->power_limit;
        energy_ = feedback->energy;
        error_code_ = feedback->error;
        ++sample_sequence_;
        tick();
    }, 8);
    connected_ = true;
}

void super_cap::start() {
    if (!connected_) throw std::logic_error("super capacitor must connect before start");
    if (started_) return;
    started_ = true;
    roboctrl::spawn(task());
}

awaitable<void> super_cap::set(bool enabled, uint16_t power_limit) {
    if (!configured_) throw std::logic_error("super capacitor is not configured");
    requested_enabled_ = enabled && !async::shutdown_requested();
    requested_power_limit_ = async::shutdown_requested() ? 0 : std::min(power_limit, info_.max_power_limit);
    last_command_ = std::chrono::steady_clock::now();
    co_return;
}

awaitable<void> super_cap::task() {
    while (true) {
        const bool fresh = std::chrono::steady_clock::now() - last_command_ <= info_.command_timeout;
        const bool enabled = super_cap_protocol::output_enabled(requested_enabled_, fresh, !offline(), error_code_);
        const auto data = super_cap_protocol::encode(enabled,
            fresh ? requested_power_limit_ : 0, info_.buffer_target);
        co_await get<io::can>(info_.can_name).send(info_.command_id, data);
        co_await wait_for(info_.resend_time);
    }
}

awaitable<void> super_cap::stop_output() {
    if (!connected_) co_return;
    co_await set(false, 0);
    const auto zero = super_cap_protocol::encode(false, 0, info_.buffer_target);
    co_await get<io::can>(info_.can_name).send(info_.command_id, zero);
}
