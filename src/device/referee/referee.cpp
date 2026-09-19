#include "device/referee/referee.h"
#include "io/serial.h"
#include <stdexcept>

using namespace roboctrl;
using namespace roboctrl::device;

bool referee::init(const info_type& info) {
    if (configured_) throw std::logic_error("referee already initialized");
    if (info.serial_name.empty() || info.offline_timeout <= std::chrono::steady_clock::duration::zero() ||
        info.ui_period < std::chrono::milliseconds{100})
        throw std::invalid_argument("referee needs serial, positive timeout and UI period >= 100 ms");
    info_ = info;
    configured_ = true;
    return true;
}
void referee::connect() {
    if (connected_) return;
    if (!configured_) throw std::logic_error("referee connect before init");
    serial_ = &get<io::serial>(info_.serial_name);
    serial_->on_raw_data([this](io::byte_span data) { receive(data); });
    connected_ = true;
}
void referee::start() {
    if (started_) return;
    if (!connected_) throw std::logic_error("referee start before connect");
    started_ = true;
    if (info_.ui_enabled) spawn(task());
}
bool referee::offline() const {
    const auto now = referee_protocol::clock::now();
    return !configured_ || !received_at_ || now < *received_at_ || now - *received_at_ > info_.offline_timeout;
}
void referee::receive(referee_protocol::bytes data) {
    parser_.feed(data, [this](const auto& frame) {
        const auto now = referee_protocol::clock::now();
        if (state_.accept(frame, now)) received_at_ = now;
        if (frame_callback_) frame_callback_(frame);
    });
}
std::optional<std::pair<std::uint16_t, std::uint16_t>> referee::ui_addresses() const {
    if (!connected_ || !info_.ui_enabled ||
        !state_.robot.fresh(referee_protocol::clock::now(), info_.offline_timeout)) return std::nullopt;
    const std::uint16_t sender = state_.robot.value.robot_id;
    const auto receiver = info_.ui_receiver_id != 0 ? std::optional{info_.ui_receiver_id} : referee_protocol::client_id(sender);
    if (!receiver) return std::nullopt;
    return std::pair{sender, *receiver};
}
awaitable<void> referee::send_graphics(std::span<const referee_protocol::graphic> items) {
    if (const auto addresses = ui_addresses()) {
        const auto frame = referee_protocol::encode_graphics(items, addresses->first, addresses->second, sequence_++);
        co_await serial_->send_raw(frame);
    }
}
awaitable<void> referee::send_text(referee_protocol::graphic item, std::string_view text) {
    if (const auto addresses = ui_addresses()) {
        const auto frame = referee_protocol::encode_text(item, text, addresses->first, addresses->second, sequence_++);
        co_await serial_->send_raw(frame);
    }
}
awaitable<void> referee::delete_graphics(std::uint8_t operation, std::uint8_t layer) {
    if (const auto addresses = ui_addresses()) {
        const auto frame = referee_protocol::encode_delete(operation, layer, addresses->first, addresses->second, sequence_++);
        co_await serial_->send_raw(frame);
    }
}
awaitable<void> referee::task() {
    referee_protocol::ui_refresh_state refresh;
    while (true) {
        if (const auto addresses = ui_addresses()) {
            const auto graphics = referee_protocol::status_graphics(ui_status_, refresh.add_required(*addresses));
            // Use the same validated address for planning and encoding. Traffic
            // without fresh robot identity must not consume the initial Add.
            const auto frame = referee_protocol::encode_graphics(
                graphics, addresses->first, addresses->second, sequence_++);
            co_await serial_->send_raw(frame);
            refresh.submitted(*addresses);
        } else {
            refresh.reset();
        }
        co_await wait_for(info_.ui_period);
    }
}
