#include "device/remote_logger.hpp"

#include <stdexcept>

namespace roboctrl::device {
remote_logger::remote_logger(const info_type& info)
    : info_{info}, peer_{asio::ip::make_address(info.address), info.port} {
    if (info.key_.empty() || info.udp_name.empty() || info.port == 0) {
        throw std::invalid_argument("invalid remote logger configuration");
    }
}

void remote_logger::connect() {
    if (!udp_) udp_ = &roboctrl::get<io::udp_server>(info_.udp_name);
}

void remote_logger::start() {
    if (!udp_) throw std::logic_error("remote logger start before connect");
    started_ = true;
}

awaitable<void> remote_logger::push_value(std::string name, double value) {
    if (!started_) co_return;
    // Repeat registration in the same datagram as every sample. Receivers can
    // start late, and loss of the first UDP frame cannot orphan later values.
    auto packet = network_protocol::encode_log_name(name);
    const auto sample = network_protocol::encode_log_value(network_protocol::log_name_id(name), value);
    packet.insert(packet.end(), sample.begin(), sample.end());
    co_await udp_->send(peer_, packet);
}

awaitable<void> remote_logger::push_console_message(std::string message) {
    if (!started_) co_return;
    const auto packet = network_protocol::encode_log_text(message);
    co_await udp_->send(peer_, packet);
}

awaitable<void> remote_logger::push_message_box(std::string message) {
    if (!started_) co_return;
    const auto packet = network_protocol::encode_log_text(message, true);
    co_await udp_->send(peer_, packet);
}
} // namespace roboctrl::device
