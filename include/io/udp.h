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
#include <string>

#include "core/async.hpp"
#include "io/base.hpp"
#include "core/logger.h"
#include "io/write_queue.hpp"

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
        using key_type = std::string;
        using owner_type = udp;

        std::string key_;
        std::string address;
        std::uint16_t port;

        const std::string& key()const{
            return key_;
        }
    };

    /**
     * @brief 构造并连接到指定远端。
     */
    udp(info_type info);

    /** @brief 启动 UDP 接收协程；重复调用不会重复启动。 */
    void start();

    /**
     * @brief 异步发送一段字节数据。
     */
    awaitable<void> send(byte_span data);

    /**
     * @brief 接收循环任务。
     */
    awaitable<void> task();

    /** @brief 返回 UDP 端点描述。 */
    inline std::string desc()const{
        return std::format("udp socket ({} to {}:{})",info_.key_,info_.address,info_.port);
    }

private:
  asio::ip::udp::socket socket_;
  write_queue write_queue_;
  info_type info_;
  std::array<std::byte,65536> buffer_;
  bool started_ {false};
};

static_assert(bare_io<udp>);
}
