#include "io/serial.h"
#include "asio/completion_condition.hpp"
#include "asio/read.hpp"
#include "core/async.hpp"
#include "io/base.hpp"
#include "utils/utils.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

using namespace roboctrl::io;

serial::serial(info_type info)
    : keyed_io_base<uint8_t>{},
      port_{roboctrl::io_context()},
      info_{info}
{
    port_.open(std::string(info.device));
    port_.set_option(asio::serial_port_base::baud_rate(info.baud_rate));
    port_.set_option(asio::serial_port_base::character_size(8));
    port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

    roboctrl::spawn(task());
}

roboctrl::awaitable<void> serial::send(uint8_t id,byte_span data)
{
    co_await asio::async_write(port_, asio::buffer(data), asio::use_awaitable);
}

roboctrl::awaitable<void> serial::read_n(size_t size){
    co_await asio::async_read(
        port_,
        asio::buffer(buffer_),
        asio::transfer_exactly(size),
        asio::use_awaitable
    );
}

roboctrl::awaitable<void> serial::task()
{
    while(true){
        uint16_t header = co_await read<uint16_t>();
        
        if(header == 0xAA55){
            uint8_t key = co_await read<uint8_t>();
            auto len = package_size(key);
            co_await read_n(len);
            dispatch(key, byte_span{buffer_.data(),len});
        }
    }
}

