#include "io/tcp.h"
#include "core/async.hpp"
#include "io/base.hpp"

#include <format>
#include <utility>

using namespace roboctrl::io;

tcp::tcp(info_type info)
    : bare_io_base{},
      socket_{roboctrl::executor()},
      info_{std::move(info)}
{
    auto endpoint = asio::ip::tcp::endpoint(
        asio::ip::make_address(info.address),
        info.port
    );
    socket_.connect(endpoint);
}

tcp::tcp(asio::ip::tcp::socket socket, std::string key)
    : bare_io_base{},
      socket_{std::move(socket)},
      info_{.name = std::move(key), .address = std::string{}, .port = 0}
{
    auto remote = socket_.remote_endpoint();
    info_.address = remote.address().to_string();
    info_.port = remote.port();
}

roboctrl::awaitable<void> tcp::send(byte_span data)
{
    co_await asio::async_write(socket_, asio::buffer(data), asio::use_awaitable);
}

roboctrl::awaitable<void> tcp::task()
{
    while(true){
        auto bytes = co_await socket_.async_read_some(asio::buffer(buffer_), asio::use_awaitable);
        dispatch(buffer_);
    }
}

tcp_server::tcp_server(info_type info)
    : acceptor_{roboctrl::get<task_context>().get_executor()},
      info_{std::move(info)}
{
    auto endpoint = asio::ip::tcp::endpoint(
        asio::ip::make_address(info_.address),
        info_.port
    );

    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::ip::tcp::acceptor::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();
}

roboctrl::awaitable<void> tcp_server::task()
{
    while(true){
        asio::ip::tcp::socket socket{acceptor_.get_executor()};
        co_await acceptor_.async_accept(socket, asio::use_awaitable);
        auto connection = make_connection(std::move(socket));
        connections_.push_back(connection);
        roboctrl::spawn(connection->task());
        on_connect_(connection);
    }
}

std::shared_ptr<tcp> tcp_server::make_connection(asio::ip::tcp::socket socket)
{
    auto remote = socket.remote_endpoint();
    auto key = std::format("{}:{}:{}:{}", info_.name, remote.address().to_string(), remote.port(), connections_.size());
    return std::make_shared<tcp>(std::move(socket), std::move(key));
}
