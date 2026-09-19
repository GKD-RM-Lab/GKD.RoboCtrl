#include "device/aim_link.hpp"

#include <stdexcept>

namespace roboctrl::device {
aim_link::aim_link(const info_type& info)
    : device_base{std::chrono::milliseconds{info.target_timeout_ms}}, info_{info},
      peer_{asio::ip::make_address(info.address), info.port} {
    if (info.key_.empty() || info.udp_name.empty() || info.port == 0 ||
        info.header == 0x37 || info.target_timeout_ms == 0 ||
        (info.wire_format != vision_wire_format::legacy_14 &&
         info.wire_format != vision_wire_format::compact_10)) {
        throw std::invalid_argument("invalid vision link configuration");
    }
}

void aim_link::connect() {
    if (udp_) return;
    udp_ = &roboctrl::get<io::udp_server>(info_.udp_name);
    udp_->on_data(peer_, [this](io::byte_span bytes) {
        if (!started_) return;
        if (const auto value = network_protocol::decode_aim(bytes, info_.header, info_.wire_format)) {
            target_.update(*value);
            tick();
        }
    });
}

void aim_link::start() {
    if (!udp_) throw std::logic_error("vision link start before connect");
    started_ = true;
}

std::optional<aim_target> aim_link::target() const {
    return target_.get(std::chrono::milliseconds{info_.target_timeout_ms});
}

awaitable<void> aim_link::send_posture(float yaw, float pitch, bool red) {
    if (!started_) co_return;
    const auto packet = network_protocol::encode_posture(info_.header, yaw, pitch, red);
    co_await udp_->send(peer_, packet);
}

navigation_link::navigation_link(const info_type& info)
    : device_base{std::chrono::milliseconds{info.target_timeout_ms}}, info_{info},
      peer_{asio::ip::make_address(info.address), info.port} {
    if (info.key_.empty() || info.udp_name.empty() || info.port == 0 || info.target_timeout_ms == 0) {
        throw std::invalid_argument("invalid navigation link configuration");
    }
}

void navigation_link::connect() {
    if (udp_) return;
    udp_ = &roboctrl::get<io::udp_server>(info_.udp_name);
    udp_->on_data(peer_, [this](io::byte_span bytes) {
        if (!started_) return;
        if (const auto value = network_protocol::decode_navigation(bytes)) {
            command_.update(*value);
            tick();
        }
    });
}

void navigation_link::start() {
    if (!udp_) throw std::logic_error("navigation link start before connect");
    started_ = true;
}

std::optional<navigation_command> navigation_link::command() const {
    return command_.get(std::chrono::milliseconds{info_.target_timeout_ms});
}

awaitable<void> navigation_link::send_status(float yaw, float hp_fraction, bool match_started) {
    if (!started_) co_return;
    const auto packet = network_protocol::encode_navigation(yaw, hp_fraction, match_started);
    co_await udp_->send(peer_, packet);
}
} // namespace roboctrl::device
