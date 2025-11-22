/**
 * @file can.h
 * @brief 基于 Linux SocketCAN 的 CAN 总线封装。
 * @details 使用 keyed IO 模型管理不同 ID 的报文处理回调。
 */
#pragma once

#include <cstdint>
#include <string_view>
#include <asio.hpp>
#include <linux/can.h>

#include "base.hpp"
#include "core/async.hpp"
#include "core/logger.h"

namespace roboctrl::io{

using can_id_type = uint32_t;

/**
 * @brief CAN 设备对象，支持根据 ID 分发回调。
 */
class can:public keyed_io_base<std::uint32_t>,public logable<can>{
public:
    /**
     * @brief CAN 初始化参数。
     */
    struct info_type{
        std::string_view can_name;

        using key_type = std::string_view;
        using owner_type = can;

        std::string_view key() const{return can_name;}
    };

    using key_type = std::uint32_t;

    /**
     * @brief 打开 CAN 设备。
     */
    can(const info_type& info);

    ~can();

    /**
     * @brief 发送裸帧。
     */
    awaitable<void> send(byte_span data);

    /**
     * @brief 发送带 CAN ID 的帧。
     */
    awaitable<void> send(can_id_type id,byte_span data);

    /**
     * @brief 接收循环任务。
     */
    awaitable<void> task();

    std::string desc()const{
        return std::format("bare can({})",info_.can_name);
    }

private:
    asio::posix::stream_descriptor stream_;
    info_type info_;
    std::array<std::byte,20> buffer_;
    ::can_frame *cf_;
    std::string can_name_;
};
}
