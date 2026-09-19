#include "io/udp_server.hpp"

#include <stdexcept>

namespace roboctrl::io {
udp_server::udp_server(const info_type& info) : info_{info}, socket_{roboctrl::executor()} {
    if (info.key_.empty()) throw std::invalid_argument("empty UDP server key");
    // Validate the address without opening a socket in the construct phase.
    asio::ip::make_address(info_.address);
}

void udp_server::connect() {
    if (connected_) return;
    const endpoint local{asio::ip::make_address(info_.address), info_.port};
    socket_.open(local.protocol());
    socket_.bind(local);
    connected_ = true;
}

void udp_server::start() {
    if (started_) return;
    if (!connected_) throw std::logic_error("UDP server start before connect");
    started_ = true;
    roboctrl::spawn(task());
}

void udp_server::stop() {
    asio::error_code ignored;
    socket_.close(ignored);
    started_ = false;
    connected_ = false;
}

awaitable<void> udp_server::send(const endpoint& peer, byte_span data) {
    if (!started_) co_return;
    if (data.empty() || data.size() > 65507 || peer.port() == 0) {
        throw std::invalid_argument("invalid UDP datagram or peer port");
    }
    if (queued_bytes_ + data.size() > 65536) {
        logger::instance().log_warn("drop UDP datagram: transmit queue full");
        co_return;
    }
    queue_.push_back({peer, {data.begin(), data.end()}});
    queued_bytes_ += data.size();
    if (!sending_) {
        sending_ = true;
        roboctrl::spawn(send_task());
    }
}

awaitable<void> udp_server::send_task() {
    while (!queue_.empty()) {
        auto packet = std::move(queue_.front());
        queue_.pop_front();
        queued_bytes_ -= packet.data.size();
        asio::error_code error;
        const auto sent = co_await socket_.async_send_to(asio::buffer(packet.data), packet.peer,
            asio::redirect_error(asio::use_awaitable, error));
        if (error || sent != packet.data.size()) {
            logger::instance().log_warn("UDP datagram write failed: {}", error.message());
        }
    }
    sending_ = false;
}

awaitable<void> udp_server::task() {
    while (started_) {
        endpoint source;
        asio::error_code error;
        const auto size = co_await socket_.async_receive_from(asio::buffer(buffer_), source,
            asio::redirect_error(asio::use_awaitable, error));
        if (error == asio::error::operation_aborted || error == asio::error::bad_descriptor) break;
        if (error) {
            logger::instance().log_warn("UDP datagram receive failed: {}", error.message());
            continue;
        }
        const auto it = callbacks_.find(source);
        if (it != callbacks_.end()) {
            it->second(make_shared_from(byte_span{buffer_.data(), size}));
        }
    }
}

} // namespace roboctrl::io
