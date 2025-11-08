#pragma once

#include <array>
#include <asio.hpp>
#include <format>
#include <span>
#include <string_view>
#include <utility>

#include "core/task_context.hpp"
#include "io/base.hpp"

namespace roboctrl::io{
class serial : public bare_io_base{
public:
    struct info_type{
        using key_type = std::string_view;
        using owner_type = serial;

        std::string_view name;
        std::string_view device;
        unsigned int baud_rate;

        task_context& context;

        std::string_view key()const{
            return name;
        }
    };

    explicit serial(info_type info);

    awaitable<void> send(byte_span data);

    awaitable<void> task();

    inline std::string desc()const{
        return std::format("serial port ({} on {} @ {}bps)",info_.name,info_.device,info_.baud_rate);
    }

private:
    asio::serial_port port_;
    info_type info_;
    std::array<std::byte,1024> buffer_;
};

static_assert(bare_io<serial>);
}
