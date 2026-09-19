#include "io/tcp.h"
#include "core/async.hpp"
#include "io/base.hpp"

#include <format>
#include <utility>

using namespace roboctrl::io;

tcp::tcp(info_type info)
    : bare_io_base{},
      socket_{roboctrl::executor()},
      write_queue_{[this](byte_span data) -> awaitable<void> {
          co_await asio::async_write(socket_, asio::buffer(data), asio::use_awaitable);
      }},
      info_{std::move(info)}
{
    auto endpoint = asio::ip::tcp::endpoint(
        asio::ip::make_address(info_.address),
        info_.port
    );
    socket_.connect(endpoint);
}

void tcp::start() {
    if (started_) {
        return;
    }
    started_ = true;
    if (auto self = weak_from_this().lock()) {
        roboctrl::spawn(run_with_lifetime(std::move(self)));
    } else {
        roboctrl::spawn(task());
    }
}

roboctrl::awaitable<void> tcp::run_with_lifetime(std::shared_ptr<tcp> self)
{
    // The shared_ptr is a coroutine parameter and therefore resides in the
    // coroutine frame until the receive loop completes.
    co_await self->task();
}

tcp::tcp(asio::ip::tcp::socket socket, std::string key)
    : bare_io_base{},
      socket_{std::move(socket)},
      write_queue_{[this](byte_span data) -> awaitable<void> {
          co_await asio::async_write(socket_, asio::buffer(data), asio::use_awaitable);
      }},
      info_{.name = std::move(key), .address = std::string{}, .port = 0}
{
    auto remote = socket_.remote_endpoint();
    info_.address = remote.address().to_string();
    info_.port = remote.port();
}

roboctrl::awaitable<void> tcp::send(byte_span data)
{
    std::shared_ptr<void> keepalive;
    if (auto self = weak_from_this().lock()) {
        keepalive = std::move(self);
    }
    co_await write_queue_.send(data, std::move(keepalive));
}

roboctrl::awaitable<void> tcp::task()
{
    try {
        while(true){
            auto bytes = co_await socket_.async_read_some(asio::buffer(buffer_), asio::use_awaitable);
            if (bytes == 0) {
                break;
            }
            dispatch(byte_span{buffer_.data(), bytes});
        }
    } catch (const asio::system_error& error) {
        if (error.code() != asio::error::eof &&
            error.code() != asio::error::operation_aborted &&
            error.code() != asio::error::connection_reset) {
            logger::instance().log_warn("tcp receive stopped: {}", error.what());
        }
    }
    notify_closed();
}

void tcp::notify_closed()
{
    if (!closed_notified_) {
        closed_notified_ = true;
        if (on_close_) {
            on_close_();
        }
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

void tcp_server::start()
{
    if (started_) {
        return;
    }
    started_ = true;
    roboctrl::spawn(task());
}

roboctrl::awaitable<void> tcp_server::task()
{
    try {
        while(true){
            asio::ip::tcp::socket socket{acceptor_.get_executor()};
            co_await acceptor_.async_accept(socket, asio::use_awaitable);
            auto connection = make_connection(std::move(socket));
            connections_.push_back(connection);
            connection->on_close([this, weak = std::weak_ptr<tcp>{connection}] {
                if (auto live = weak.lock()) {
                    remove_connection(live);
                }
            });
            connection->start();
            on_connect_(connection);
        }
    } catch (const asio::system_error& error) {
        if (error.code() != asio::error::operation_aborted) {
            logger::instance().log_warn("tcp server stopped: {}", error.what());
        }
    }
}

void tcp_server::remove_connection(const std::shared_ptr<tcp>& connection)
{
    std::erase(connections_, connection);
}

std::shared_ptr<tcp> tcp_server::make_connection(asio::ip::tcp::socket socket)
{
    auto remote = socket.remote_endpoint();
    auto key = std::format("{}:{}:{}:{}", info_.name, remote.address().to_string(), remote.port(), connections_.size());
    return std::make_shared<tcp>(std::move(socket), std::move(key));
}
