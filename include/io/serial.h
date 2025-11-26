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
#include <string_view>
#include <utility>

#include "core/async.hpp"
#include "io/base.hpp"
#include "utils/concepts.hpp"
#include "utils/utils.hpp"

namespace roboctrl::io{


/**
 * @brief 串口设备对象。
 */
class serial : public keyed_io_base<uint8_t>{
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

    using key_type = std::uint8_t;

    /**
     * @brief 打开并配置串口。
     */
    explicit serial(info_type info);

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
    awaitable<void> read_n(size_t size);

    template<utils::package pkg_type>
    awaitable<pkg_type> read(){
        co_await read_n(sizeof(pkg_type));
        co_return utils::from_bytes<pkg_type>(byte_span{buffer_.data(),sizeof(pkg_type)});
    }

private:
    asio::serial_port port_;
    info_type info_;
    std::array<std::byte,1024> buffer_;

    static constexpr uint16_t header_magic = 0xAA55;
};

static_assert(keyed_io<serial>);
}
