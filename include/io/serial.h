/**
 * @file serial.h
 * @brief 串口裸 IO 封装。
 * @details 针对 POSIX 平台的串口封装，提供异步读写接口。
 */
#pragma once

#include <array>
#include <asio.hpp>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "core/async.hpp"
#include "io/base.hpp"
#include "utils/concepts.hpp"
#include "utils/utils.hpp"
#include "io/write_queue.hpp"
#include "core/logger.h"

namespace roboctrl::io{


/**
 * @brief 串口设备对象。
 */
class serial : public keyed_io_base<uint8_t>, public logable<serial>{
public:
    /**
     * @brief 串口初始化参数。
     */
    struct info_type{
        using key_type = std::string;
        using owner_type = serial;

        std::string name;
        std::string device;
        unsigned int baud_rate;

        const std::string& key()const{
            return name;
        }
    };

    using key_type = std::uint8_t;

    /**
     * @brief 打开并配置串口。
     */
    explicit serial(info_type info);

    void start();

    /**
     * @brief 发送字节数据。
     */
    awaitable<void> send(key_type key,byte_span data);

    /**
     * @brief 接收循环任务。
     */
    awaitable<void> task();

    inline std::string desc()const{
        return std::format("serial port ({} on {} @ {}bps)",info_.name,info_.device,info_.baud_rate);
    }
private:
    void process_receive_buffer();

private:
    asio::serial_port port_;
    write_queue write_queue_;
    info_type info_;
    std::array<std::byte,256> read_buffer_;
    std::vector<std::byte> receive_buffer_;
    bool started_ {false};

    // 兼容当前 Linux/RM 端的小端线序：数值 0xAA55 在线上为 55 AA。
    static constexpr std::array<std::byte, 2> header_bytes{
        std::byte{0x55}, std::byte{0xAA}};
};

static_assert(keyed_io<serial>);
}
