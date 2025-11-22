#include "io/serial.h"
#include "core/async.hpp"
#include "io/base.hpp"

#include <memory>
#include <string>

using namespace roboctrl::io;

serial::serial(info_type info)
    : bare_io_base{},
      port_{roboctrl::io_context()},
      info_{info}
{
    port_.open(std::string(info.device));
    port_.set_option(asio::serial_port_base::baud_rate(info.baud_rate));
    port_.set_option(asio::serial_port_base::character_size(8));
    port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
}

roboctrl::awaitable<void> serial::send(byte_span data)
{
    co_await asio::async_write(port_, asio::buffer(data), asio::use_awaitable);
}

roboctrl::awaitable<void> serial::task()
{
    while(true){
        auto bytes = co_await port_.async_read_some(asio::buffer(buffer_), asio::use_awaitable);
        dispatch(buffer_);
    }
}
