/**
 * @file can.h
 * @brief 基于 Linux SocketCAN 的 CAN 总线封装。
 * @details 使用 keyed IO 模型管理不同 ID 的报文处理回调。
 */
#pragma once

#include <cstdint>
#include <string>
#include <asio.hpp>

#include "base.hpp"
#include "core/async.hpp"
#include "core/logger.h"
#include "write_queue.hpp"

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
        /// 项目内稳定名称，供电机等其他组件引用。
        std::string name;
        /// 实际 SocketCAN 接口名，例如 can0 或 CAN_CHASSIS。
        std::string interface_name;

        using key_type = std::string;
        using owner_type = can;

        const std::string& key() const{return name;}
    };

    using key_type = std::uint32_t;

    /**
     * @brief 打开 CAN 设备。
     */
    can(const info_type& info);

    ~can() = default;

    /** @brief 启动接收协程；重复调用不会重复启动。 */
    void start();

    /**
     * @brief 发送带 CAN ID 的帧。
     * @details 同一 ID 尚未写出的帧只保留最新值；正在写出的帧无法撤回。
     */
    awaitable<void> send(can_id_type id,byte_span data);

    /**
     * @brief 接收循环任务。
     */
    awaitable<void> task();

    /** @brief 返回包含逻辑名称和 Linux 接口名的可读描述。 */
    std::string desc()const{
        return std::format("bare can({} on {})",info_.name, info_.interface_name);
    }

private:
    asio::posix::stream_descriptor stream_;
    write_queue write_queue_;
    info_type info_;
    std::array<std::byte,20> buffer_;
    std::string interface_name_;
    bool started_ {false};
};

static_assert(keyed_io<can>);
}
