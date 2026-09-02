#pragma once

#include <asio.hpp>
#include <asio/use_awaitable.hpp>

#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "core/async.hpp"
#include "core/logger.h"

namespace roboctrl::io {

/**
 * @brief 将同一异步流上的写操作串行化。
 *
 * 单线程事件循环不能保证多个协程发起的异步写操作不会交错。
 * 该队列复制待发送数据，并由唯一 writer 协程按入队顺序发送。
 */
class write_queue {
public:
    using writer_type = std::function<awaitable<void>(std::span<const std::byte>)>;

    explicit write_queue(writer_type writer) : writer_{std::move(writer)} {}

    write_queue(const write_queue&) = delete;
    write_queue& operator=(const write_queue&) = delete;

    awaitable<void> send(std::span<const std::byte> data,
                         std::shared_ptr<void> keepalive = {}) {
        if (data.empty()) {
            co_return;
        }

        if (queued_bytes_ + data.size() > max_queued_bytes_) {
            logger::instance().log_warn(
                "drop asynchronous write because the queue is full ({} bytes)", queued_bytes_);
            co_return;
        }

        queue_.push_back(item{
            std::make_shared<std::vector<std::byte>>(data.begin(), data.end()),
            std::move(keepalive)});
        queued_bytes_ += data.size();
        if (!writing_) {
            writing_ = true;
            roboctrl::spawn(drain());
        }
        co_return;
    }

private:
    awaitable<void> drain() {
        try {
            while (!queue_.empty()) {
                auto frame = std::move(queue_.front());
                queue_.pop_front();
                queued_bytes_ -= frame.data->size();
                co_await writer_(std::span<const std::byte>{frame.data->data(), frame.data->size()});
            }
        } catch (const asio::system_error& error) {
            if (error.code() != asio::error::operation_aborted) {
                logger::instance().log_error(
                    "asynchronous write failed: {}", error.what());
            }
            queue_.clear();
            queued_bytes_ = 0;
        } catch (const std::exception& error) {
            queue_.clear();
            queued_bytes_ = 0;
            roboctrl::logger::instance().log_error(
                "asynchronous write failed: {}", error.what());
        } catch (...) {
            queue_.clear();
            queued_bytes_ = 0;
            roboctrl::logger::instance().log_error(
                "asynchronous write failed with an unknown exception");
        }
        writing_ = false;
    }

    writer_type writer_;
    struct item {
        std::shared_ptr<std::vector<std::byte>> data;
        std::shared_ptr<void> keepalive;
    };

    std::deque<item> queue_;
    std::size_t queued_bytes_{0};
    bool writing_{false};

    static constexpr std::size_t max_queued_bytes_ = 64 * 1024;
};

} // namespace roboctrl::io
