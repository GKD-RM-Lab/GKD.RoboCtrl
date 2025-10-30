#pragma once

#include "base.hpp"
#include "core/task_context.hpp"
#include <cstdint>
#include <string_view>
#include <asio.hpp>
#include <linux/can.h>

#include "core/logger.hpp"

namespace roboctrl::io{

using can_id_type = uint32_t;

class can_io:public keyed_io_base<std::uint32_t>{
public:
    struct info_type{
        std::string can_name;
        core::task_context::task_context& context;


        using key_type = std::string;
        using owner_type = can_io;

        std::string key() const{return can_name;}
    };

    can_io(const info_type& info);

    ~can_io();

    awaitable<void> send(byte_span data);

    awaitable<void> send(can_id_type id,byte_span data);

    awaitable<void> task();

    std::string desc()const{
        return std::format("bare can({})",info_.can_name);
    }

private:
    asio::posix::stream_descriptor stream_;
    info_type info_;
    std::array<std::byte,20> buffer_;
    ::can_frame *cf_;
};

static_assert(owner<can_io>);



}