/**
 * @file serial.h
 * @brief 串口裸 IO 封装。
 * @details 针对 POSIX 平台的串口封装，提供异步读写接口。
 */
#pragma once

#include <array>
#include <asio.hpp>
#include <format>
#include <span>
#include <string_view>
#include <utility>

#include "core/async.hpp"
#include "io/base.hpp"

namespace roboctrl::io{

/**
 * @brief 串口设备对象。
 */
class serial : public bare_io_base{
public:
    /**
     * @brief 串口初始化参数。
     */
    struct info_type{
        using key_type = std::string_view;
        using owner_type = serial;

        std::string_view name;
        std::string_view device;
        unsigned int baud_rate;

        std::string_view key()const{
            return name;
        }
    };

    /**
     * @brief 打开并配置串口。
     */
    explicit serial(info_type info);

    /**
     * @brief 发送字节数据。
     */
    awaitable<void> send(byte_span data);

    /**
     * @brief 接收循环任务。
     */
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
