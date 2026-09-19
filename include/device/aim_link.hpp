#pragma once

#include "device/base.hpp"
#include "device/network_protocol.hpp"
#include "io/udp_server.hpp"

namespace roboctrl::device {

class aim_link : public device_base {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = aim_link;
        std::string key_;
        std::string udp_name;
        std::string address{"127.0.0.1"};
        std::uint16_t port{11453};
        std::uint8_t header{0x6a};
        std::uint32_t target_timeout_ms{100};
        vision_wire_format wire_format{vision_wire_format::legacy_14};
        const std::string& key() const { return key_; }
    };
    explicit aim_link(const info_type& info);
    void connect();
    void start();
    std::optional<aim_target> target() const;
    awaitable<void> send_posture(float yaw, float pitch, bool red);
    std::string desc() const { return std::format("vision link {}", info_.key_); }

private:
    info_type info_;
    io::udp_server* udp_{};
    io::udp_server::endpoint peer_;
    network_protocol::fresh_value<aim_target> target_;
    bool started_{};
};

class navigation_link : public device_base {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = navigation_link;
        std::string key_;
        std::string udp_name;
        std::string address{"127.0.0.1"};
        std::uint16_t port{11456};
        std::uint32_t target_timeout_ms{200};
        const std::string& key() const { return key_; }
    };
    explicit navigation_link(const info_type& info);
    void connect();
    void start();
    std::optional<navigation_command> command() const;
    awaitable<void> send_status(float yaw, float hp_fraction, bool match_started);
    std::string desc() const { return std::format("navigation link {}", info_.key_); }

private:
    info_type info_;
    io::udp_server* udp_{};
    io::udp_server::endpoint peer_;
    network_protocol::fresh_value<navigation_command> command_;
    bool started_{};
};
} // namespace roboctrl::device
