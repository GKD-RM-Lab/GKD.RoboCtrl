#include "device/motor/m9025.h"
#include "io/can.h"

using namespace roboctrl;
using namespace roboctrl::device;

m9025::m9025(const info_type& info)
    : motor_base{info.offline_timeout, info.radius}, info_{info}, pid_{info.pid_params} {
    if (!valid_configuration(info)) throw std::invalid_argument("invalid M9025 configuration or feedback scale");
}

void m9025::connect() {
    if (connected_) return;
    get<io::can>(info_.can_name).on_data(0x140 + info_.id, [this](io::byte_span data) {
        const auto feedback = motor_protocol::decode_m9025(data);
        if (!feedback) return;
        angle_ = info_.direction * 2.f * Pi_f * feedback->encoder / info_.encoder_counts_per_turn;
        angle_speed_ = info_.direction * feedback->speed_raw * info_.speed_rad_per_count;
        torque_ = info_.direction * feedback->current_raw;
        tick();
    }, 8);
    connected_ = true;
}

void m9025::start() {
    if (!connected_) throw std::logic_error("M9025 must connect before start");
    if (started_) return;
    started_ = true;
    roboctrl::spawn(task());
}

awaitable<void> m9025::set(fp32 speed) { co_await set_angle_speed(speed / radius_); }

awaitable<void> m9025::set_angle_speed(fp32 speed) {
    if (direct_current_) pid_.clean();
    direct_current_ = false;
    pid_.set_target(enabled_ && std::isfinite(speed) ? speed : 0.f);
    co_return;
}

awaitable<void> m9025::set_current(fp32 command) {
    if (!direct_current_) pid_.clean();
    direct_current_ = true;
    current_ = enabled_ && !offline() && std::isfinite(command)
        ? std::clamp(command, -max_current(), max_current()) : 0.f;
    co_return;
}

void m9025::disable() {
    enabled_ = false;
    current_ = 0.f;
    pid_.clean();
}

void m9025::set_enabled(bool enabled) {
    if (enabled && !async::shutdown_requested()) enabled_ = true;
    else disable();
}

awaitable<void> m9025::task() {
    while (true) {
        if (!enabled_ || offline()) {
            current_ = 0.f;
            pid_.clean();
        } else if (!direct_current_) {
            pid_.update(angle_speed(), std::chrono::duration<fp32>(info_.control_time).count());
            current_ = pid_.state();
        }
        const auto data = motor_protocol::encode_m9025_current(static_cast<int16_t>(info_.direction * current()));
        co_await get<io::can>(info_.can_name).send(0x140 + info_.id, data);
        co_await wait_for(info_.control_time);
    }
}

awaitable<void> m9025::stop_output() {
    disable();
    const auto zero = motor_protocol::encode_m9025_current(0);
    co_await get<io::can>(info_.can_name).send(0x140 + info_.id, zero);
}
