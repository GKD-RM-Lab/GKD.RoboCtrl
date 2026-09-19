#pragma once

#include "device/network_protocol.hpp"
#include "io/udp_server.hpp"

namespace roboctrl::device {

/** Legacy log datagrams; telemetry is outbound and never changes control state. */
class remote_logger {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = remote_logger;
        std::string key_;
        std::string udp_name;
        std::string address{"127.0.0.1"};
        std::uint16_t port{8080};
        const std::string& key() const { return key_; }
    };
    explicit remote_logger(const info_type& info);
    void connect();
    void start();
    awaitable<void> push_value(std::string name, double value);
    awaitable<void> push_console_message(std::string message);
    awaitable<void> push_message_box(std::string message);
    std::string desc() const { return std::format("remote logger {}", info_.key_); }
private:
    info_type info_;
    io::udp_server* udp_{};
    io::udp_server::endpoint peer_;
    bool started_{};
};
} // namespace roboctrl::device
