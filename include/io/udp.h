/**
 * @file udp.h
 * @brief UDP 裸 IO 封装。
 * @details 基于 Asio 实现的简易 UDP socket，提供发送、接收协程任务。
 */
#pragma once
#include <asio.hpp>
#include <format>
#include <string_view>
#include <span>

#include "core/async.hpp"
#include "io/base.hpp"
#include "core/logger.h"

namespace roboctrl::io{

/**
 * @brief UDP 通信端点。
 */
class udp : public bare_io_base{
public:
    /**
     * @brief UDP 初始化参数。
     */
    struct info_type{
        using key_type = std::string_view;
        using owner_type = udp;

        std::string_view key_;
        std::string_view address;
        int port;

        std::string_view key()const{
            return key_;
        }
    };

    /**
     * @brief 构造并连接到指定远端。
     */
    udp(info_type info);

    /**
     * @brief 异步发送一段字节数据。
     */
    awaitable<void> send(byte_span data);

    /**
     * @brief 接收循环任务。
     */
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
