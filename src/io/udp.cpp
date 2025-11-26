#include "io/udp.h"
#include "core/async.hpp"
#include "io/base.hpp"

using namespace roboctrl::io;

udp::udp(info_type info)
    : bare_io_base{},
    socket_{roboctrl::executor()},
    info_{info}
{
    auto endpoint = asio::ip::udp::endpoint(asio::ip::make_address(info.address),info.port);
    socket_.connect(endpoint);
    
    roboctrl::spawn(task());
}

roboctrl::awaitable<void> udp::send(byte_span data)
{
    co_await socket_.async_send(asio::buffer(data),asio::use_awaitable);
}

roboctrl::awaitable<void> udp::task()
{
    while(true){
        auto bytes = co_await socket_.async_receive(asio::buffer(buffer_),asio::use_awaitable);
        dispatch(byte_span{buffer_.data(),bytes});
    }
}