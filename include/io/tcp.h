/**
 * @file tcp.h
 * @brief TCP 客户端与服务器封装。
 * @details 提供面向裸字节的 TCP socket、及其监听端，用于简化异步通信流程。
 */
#pragma once

#include <asio.hpp>
#include <array>
#include <cstdint>
#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/async.hpp"
#include "io/base.hpp"
#include "utils/callback.hpp"
#include "io/write_queue.hpp"

namespace roboctrl::io{

/**
 * @brief TCP 客户端套接字。
 */
class tcp : public bare_io_base, public std::enable_shared_from_this<tcp>{
public:
    /**
     * @brief TCP 连接参数。
     */
    struct info_type{
        using key_type = std::string;
        using owner_type = tcp;

        std::string name;       ///< TCP 连接名称
        std::string address;    ///< TCP 连接地址
        std::uint16_t port;     ///< TCP 端口

        const std::string& key()const{
            return name;
        }
    };

    /**
     * @brief 通过连接信息构造客户端。
     */
    explicit tcp(info_type info);
    /**
     * @brief 由已有 `asio::ip::tcp::socket` 包装。
     */
    tcp(asio::ip::tcp::socket socket, std::string key);

    /** @brief 启动客户端接收协程；重复调用不会重复启动。 */
    void start();

    /**
     * @brief 发送字节数据。
     */
    awaitable<void> send(byte_span data);

    /**
     * @brief 接收循环任务。
     */
    awaitable<void> task();

    /** @brief 注册连接关闭回调，仅触发一次。 */
    void on_close(std::function<void()> callback) { on_close_ = std::move(callback); }

    /** @brief 返回客户端连接描述。 */
    inline std::string desc()const{
        return std::format("tcp socket (<{}> to {}:{})",info_.name,info_.address,info_.port);
    }

private:
    static awaitable<void> run_with_lifetime(std::shared_ptr<tcp> self);

    asio::ip::tcp::socket socket_;
    write_queue write_queue_;
    info_type info_;
    std::array<std::byte,1024> buffer_;
    bool started_ {false};
    std::function<void()> on_close_;
    bool closed_notified_ {false};

    void notify_closed();
};

static_assert(bare_io<tcp>);

/**
 * @brief TCP 监听服务器，负责接受并管理多连接。
 * @details 在创建后，tcp_server 会监听指定端口，当新连接建立时，会创建新 roboctrl::io::tcp 的调用通过 on_connect 方法注册的回调函数
 */
class tcp_server{
public:
    /**
     * @brief 服务器初始化参数。
     */
    struct info_type{
        using key_type = std::string;
        using owner_type = tcp_server;

        std::string name;                   ///< 服务器名称
        std::string address;                ///< 监听地址
        std::uint16_t port;                 ///< 监听端口

        const std::string& key()const{
            return name;
        }
    };

    /**
     * @brief 构造监听器并立即开始监听。
     */
    explicit tcp_server(info_type info);

    /** @brief 启动服务器监听协程；重复调用不会重复启动。 */
    void start();

    /**
     * @brief 接受连接的长任务。
     */
    awaitable<void> task();

    /**
     * @brief 注册协程回调，在新连接建立时触发。
     */
    void on_connect(std::function<awaitable<void>(std::shared_ptr<tcp>)> callback){
        on_connect_.add(std::move(callback));
    }

    /**
     * @brief 注册同步回调，在新连接建立时触发。
     */
    void on_connect(std::function<void(std::shared_ptr<tcp>)> callback){
        on_connect_.add(std::move(callback));
    }

    /** @brief 返回服务器监听描述。 */
    inline std::string desc()const{
        return std::format("tcp server (<{}> listening on {}:{})",info_.name,info_.address,info_.port);
    }

private:
    std::shared_ptr<tcp> make_connection(asio::ip::tcp::socket socket);

    asio::ip::tcp::acceptor acceptor_;
    info_type info_;
    callback<std::shared_ptr<tcp>> on_connect_;
    std::vector<std::shared_ptr<tcp>> connections_;
    bool started_ {false};

    void remove_connection(const std::shared_ptr<tcp>& connection);
};
}
