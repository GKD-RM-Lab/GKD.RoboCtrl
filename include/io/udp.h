#pragma once
#include <asio.hpp>
#include <format>
#include <string_view>
#include <span>

#include "core/task_context.hpp"
#include "io/base.hpp"
#include "core/logger.h"

namespace roboctrl::io{
class udp : public bare_io_base{
public:
    struct info_type{
        using key_type = std::string_view;
        using owner_type = udp;

        std::string_view key_;
        std::string_view address;
        int port;

        task_context& context;

        std::string_view key()const{
            return key_;
        }
    };

    udp(info_type info);

    awaitable<void> send(byte_span data);

    awaitable<void> task();

    inline std::string desc()const{
        return std::format("udp socket ({} to {}:{})",info_.key_,info_.address,info_.port);
    }

private:
  asio::ip::udp::socket socket_;
  info_type info_;
  std::array<std::byte,1024> buffer_;
};

static_assert(bare_io<udp>);
}