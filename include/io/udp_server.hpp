#pragma once

#include <asio.hpp>
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "io/base.hpp"

namespace roboctrl::io {

/** UDP data transport with explicit receive peers and one owning send queue. */
class udp_server {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = udp_server;
        std::string key_;
        std::string address{"0.0.0.0"};
        std::uint16_t port{11451};
        const std::string& key() const { return key_; }
    };
    using endpoint = asio::ip::udp::endpoint;

    explicit udp_server(const info_type& info);
    void connect();
    void start();
    void stop();
    endpoint local_endpoint() const { return socket_.local_endpoint(); }

    template<typename Fn>
    void on_data(const endpoint& peer, Fn fn) {
        callbacks_[peer].add([fn = std::move(fn)](data_ptr data) mutable -> auto {
            return fn(byte_span{data->data(), data->size()});
        });
    }

    awaitable<void> send(const endpoint& peer, byte_span data);
    awaitable<void> task();
    std::string desc() const { return std::format("udp server {}", info_.key_); }

private:
    awaitable<void> send_task();
    struct datagram {
        endpoint peer;
        std::vector<std::byte> data;
    };
    info_type info_;
    asio::ip::udp::socket socket_;
    std::array<std::byte, 65536> buffer_{};
    std::map<endpoint, callback<data_ptr>> callbacks_;
    std::deque<datagram> queue_;
    std::size_t queued_bytes_{};
    bool connected_{};
    bool started_{};
    bool sending_{};
};
} // namespace roboctrl::io
