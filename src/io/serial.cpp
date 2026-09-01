#include "io/serial.h"
#include "asio/completion_condition.hpp"
#include "asio/read.hpp"
#include "core/async.hpp"
#include "io/base.hpp"
#include "utils/utils.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
}

void serial::start() {
    if (started_) {
        return;
    }
    started_ = true;
    roboctrl::spawn(task());
}

roboctrl::awaitable<void> serial::send(uint8_t id,byte_span data)
{
    std::vector<std::byte> frame(sizeof(header_magic) + sizeof(id) + data.size());
    std::memcpy(frame.data(), &header_magic, sizeof(header_magic));
    frame[sizeof(header_magic)] = static_cast<std::byte>(id);
    std::memcpy(frame.data() + sizeof(header_magic) + sizeof(id), data.data(), data.size());
    co_await asio::async_write(port_, asio::buffer(frame), asio::use_awaitable);
}

roboctrl::awaitable<void> serial::read_n(size_t size){
    if (size > buffer_.size()) {
        throw std::length_error("serial frame exceeds receive buffer");
    }
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
        
        if(header == header_magic){
            uint8_t key = co_await read<uint8_t>();
            const auto len = package_size(key);
            if (!len) {
                continue;
            }
            co_await read_n(*len);
            dispatch(key, byte_span{buffer_.data(),*len});
        }
    }
}
