#include "device/motor/j6006.h"
#include "io/can.h"

using namespace roboctrl;
using namespace roboctrl::device;

j6006::j6006(const info_type& info)
    : motor_base{info.offline_timeout, info.radius}, info_{info} {
    if (!valid_configuration(info)) throw std::invalid_argument("invalid J6006 configuration");
}

void j6006::connect() {
    if (connected_) return;
    get<io::can>(info_.can_name).on_data(info_.master_id, [this](io::byte_span data) {
        const auto feedback = motor_protocol::decode_j6006(data, info_.id, info_.feedback_range);
        if (!feedback) return;
        status_ = feedback->status;
        if (status_ >= 8) fault_latched_ = true;
        encoder_raw_ = feedback->position_raw;
        angle_ = info_.direction * feedback->position_rad;
        angle_speed_ = info_.direction * feedback->velocity_rad_s;
        torque_ = info_.direction * feedback->torque_nm;
        if (status_ != 1) target_speed_ = 0.f;
        tick();
    }, 8);
    connected_ = true;
}

void j6006::start() {
    if (!connected_) throw std::logic_error("J6006 must connect before start");
    if (started_) return;
    started_ = true;
    roboctrl::spawn(task());
}

awaitable<void> j6006::set(fp32 speed) { co_await set_angle_speed(speed / radius_); }

awaitable<void> j6006::set_angle_speed(fp32 speed) {
    target_speed_ = enabled_ && !fault_latched_ && !offline() && status_ == 1 && std::isfinite(speed)
        ? std::clamp(speed, -info_.max_speed, info_.max_speed) : 0.f;
    co_return;
}

void j6006::disable() {
    enabled_ = false;
    target_speed_ = 0.f;
    fault_latched_ = false;
}

void j6006::set_enabled(bool enabled) {
    if (enabled) enabled_ = true;
    else disable();
}

awaitable<void> j6006::task() {
    auto last_special = std::chrono::steady_clock::time_point::min();
    bool first = true;
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        // Retry the requested enable state, but never clear a reported fault.
        const bool request_enable = enabled_ && !fault_latched_ && (status_ == 0 || status_ == 1 || status_ == 0xff);
        const bool transition = enabled_ != previous_enabled_;
        if (first || transition || now - last_special >= std::chrono::milliseconds{100}) {
            const auto data = motor_protocol::encode_j6006_enabled(request_enable);
            co_await get<io::can>(info_.can_name).send(0x200 + info_.id, data);
            last_special = now;
            first = false;
            previous_enabled_ = enabled_;
        }
        if (offline()) target_speed_ = 0.f;
        const auto data = motor_protocol::encode_j6006_velocity(info_.direction * command_speed());
        co_await get<io::can>(info_.can_name).send(0x200 + info_.id, data);
        co_await wait_for(info_.control_time);
    }
}
