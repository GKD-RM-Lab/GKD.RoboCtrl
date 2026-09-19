#include "io/udp.h"
#include "core/async.hpp"
#include "io/base.hpp"

#include <stdexcept>

using namespace roboctrl::io;

udp::udp(info_type info)
    : bare_io_base{},
    socket_{roboctrl::executor()},
    write_queue_{[this](byte_span data) -> awaitable<void> {
        const auto sent = co_await socket_.async_send(asio::buffer(data), asio::use_awaitable);
        if (sent != data.size()) {
            throw std::runtime_error("short UDP datagram write");
        }
    }},
    info_{info}
{
    auto endpoint = asio::ip::udp::endpoint(asio::ip::make_address(info.address),info.port);
    socket_.connect(endpoint);
}

void udp::start() {
    if (started_) {
        return;
    }
    started_ = true;
    roboctrl::spawn(task());
}

roboctrl::awaitable<void> udp::send(byte_span data)
{
    co_await write_queue_.send(data);
}

roboctrl::awaitable<void> udp::task()
{
    try {
        while(true){
            auto bytes = co_await socket_.async_receive(asio::buffer(buffer_),asio::use_awaitable);
            dispatch(byte_span{buffer_.data(),bytes});
        }
    } catch (const asio::system_error& error) {
        if (error.code() != asio::error::operation_aborted) {
            roboctrl::logger::instance().log_warn("udp receive stopped: {}", error.what());
        }
    }
}
