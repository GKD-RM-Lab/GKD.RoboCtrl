#pragma once

#include <asio.hpp>
#include <array>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/task_context.hpp"
#include "io/base.hpp"
#include "utils/callback.hpp"

namespace roboctrl::io{
class tcp_io : public bare_io_base{
public:
    struct info_type{
        using key_type = std::string_view;
        using owner_type = tcp_io;

        std::string key_;
        std::string address;
        std::uint16_t port;

        task_context& context;

        std::string_view key()const{
            return key_;
        }
    };

    explicit tcp_io(info_type info);
    tcp_io(task_context& context, asio::ip::tcp::socket socket, std::string key);

    awaitable<void> send(byte_span data);

    awaitable<void> task();

    inline std::string desc()const{
        return std::format("tcp socket (<{}> to {}:{})",info_.key_,info_.address,info_.port);
    }

private:
    asio::ip::tcp::socket socket_;
    info_type info_;
    std::array<std::byte,1024> buffer_;
};

static_assert(bare_io<tcp_io>);

class tcp_server{
public:
    struct info_type{
        using key_type = std::string_view;
        using owner_type = tcp_server;

        std::string key_;
        std::string address;
        std::uint16_t port;
        task_context& context;

        std::string_view key()const{
            return key_;
        }
    };

    explicit tcp_server(info_type info);

    awaitable<void> task();

    void on_connect(std::function<awaitable<void>(std::shared_ptr<tcp_io>)> callback){
        on_connect_.add(std::move(callback));
    }

    void on_connect(std::function<void(std::shared_ptr<tcp_io>)> callback){
        on_connect_.add(std::move(callback));
    }

    inline std::string desc()const{
        return std::format("tcp server (<{}> listening on {}:{})",info_.key_,info_.address,info_.port);
    }

private:
    std::shared_ptr<tcp_io> make_connection(asio::ip::tcp::socket socket);

    asio::ip::tcp::acceptor acceptor_;
    info_type info_;
    callback<std::shared_ptr<tcp_io>> on_connect_;
    std::vector<std::shared_ptr<tcp_io>> connections_;
};
}
