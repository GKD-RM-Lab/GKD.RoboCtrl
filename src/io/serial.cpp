#include "io/serial.h"
#include "core/async.hpp"
#include "io/base.hpp"
#include "utils/utils.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

using namespace roboctrl::io;

serial::serial(info_type info)
    : keyed_io_base<uint8_t>{},
      port_{roboctrl::io_context()},
      write_queue_{[this](byte_span data) -> awaitable<void> {
          co_await asio::async_write(port_, asio::buffer(data), asio::use_awaitable);
      }},
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
    if (data.size() > 1024) {
        throw std::invalid_argument("serial payload cannot exceed 1024 bytes");
    }

    std::vector<std::byte> frame;
    frame.reserve(header_bytes.size() + sizeof(id) + data.size());
    frame.insert(frame.end(), header_bytes.begin(), header_bytes.end());
    frame.push_back(static_cast<std::byte>(id));
    frame.insert(frame.end(), data.begin(), data.end());
    co_await write_queue_.send(frame);
}

void serial::process_receive_buffer()
{
    constexpr std::size_t header_size = header_bytes.size() + sizeof(std::uint8_t);
    while (receive_buffer_.size() >= header_size) {
        if (receive_buffer_[0] != header_bytes[0] || receive_buffer_[1] != header_bytes[1]) {
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        const auto key = std::to_integer<std::uint8_t>(receive_buffer_[2]);
        const auto len = package_size(key);
        if (!len) {
            log_warn("drop serial frame with unknown key {}", key);
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        if (*len > 1024) {
            log_warn("drop serial key {} with oversized payload {}", key, *len);
            receive_buffer_.erase(receive_buffer_.begin());
            continue;
        }

        const auto frame_size = header_size + *len;
        if (receive_buffer_.size() < frame_size) {
            return;
        }

        dispatch(key, byte_span{receive_buffer_.data() + header_size, *len});
        receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + frame_size);
    }
}

roboctrl::awaitable<void> serial::task()
{
    try {
        while(true){
            const auto bytes = co_await port_.async_read_some(
                asio::buffer(read_buffer_), asio::use_awaitable);
            receive_buffer_.insert(receive_buffer_.end(), read_buffer_.begin(), read_buffer_.begin() + bytes);
            process_receive_buffer();
        }
    } catch (const asio::system_error& error) {
        if (error.code() != asio::error::operation_aborted) {
            log_warn("serial receive stopped: {}", error.what());
        }
    }
}
