#include "io/udp.h"
#include "core/task_context.hpp"
#include "io/base.hpp"

using namespace roboctrl::io;

udp::udp(info_type info)
    : bare_io_base{info.context},
    socket_{info.context.get_executor()},
    info_{info}
{
    auto endpoint = asio::ip::udp::endpoint(asio::ip::make_address(info.address),info.port);
    socket_.connect(endpoint);
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