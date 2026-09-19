#pragma once

#include <asio.hpp>
#include <asio/use_awaitable.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/async.hpp"
#include "core/logger.h"

namespace roboctrl::io {

/**
 * @brief 写队列无法接纳新的独立报文。
 */
class write_queue_full final : public std::runtime_error {
public:
    explicit write_queue_full(std::size_t queued_bytes)
        : std::runtime_error{
              "asynchronous write queue is full (" +
              std::to_string(queued_bytes) + " bytes queued)"} {}
};

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
        enqueue(data, std::move(keepalive), std::nullopt);
        return completed();
    }

    /**
     * @brief 按 key 入队，并用新数据替换同 key 的尚未发送项。
     *
     * 正在执行的写操作已经从队列弹出，无法撤回；该操作保证其后至多保留一个
     * 同 key 的 pending 项。替换会在容量检查前完成，因此队列已满时同 key 的
     * 安全停机帧仍可覆盖旧控制帧。
     */
    awaitable<void> send_latest(std::uint64_t key,
                                std::span<const std::byte> data,
                                std::shared_ptr<void> keepalive = {}) {
        enqueue(data, std::move(keepalive), key);
        return completed();
    }

    /**
     * @brief writer 是否已经失败。
     * @details 失败会被锁存；后续 send/send_latest 会重抛原异常，恢复需要重建 IO。
     */
    [[nodiscard]] bool failed() const noexcept {
        return write_failure_ != nullptr;
    }

private:
    static awaitable<void> completed() {
        co_return;
    }

    void enqueue(std::span<const std::byte> data,
                 std::shared_ptr<void> keepalive,
                 std::optional<std::uint64_t> replace_key) {
        if (write_failure_) {
            std::rethrow_exception(write_failure_);
        }
        if (data.empty()) {
            return;
        }

        if (replace_key) {
            const auto first = std::find_if(queue_.begin(), queue_.end(), [&](const item& queued) {
                return queued.replace_key == replace_key;
            });
            if (first != queue_.end()) {
                std::size_t matching_bytes = 0;
                for (const auto& queued : queue_) {
                    if (queued.replace_key == replace_key) {
                        matching_bytes += queued.data->size();
                    }
                }
                const auto retained_bytes = queued_bytes_ - matching_bytes;
                if (data.size() > max_queued_bytes_ - retained_bytes) {
                    throw write_queue_full{queued_bytes_};
                }

                auto replacement =
                    std::make_shared<std::vector<std::byte>>(data.begin(), data.end());
                queued_bytes_ -= first->data->size();
                first->data = std::move(replacement);
                first->keepalive = std::move(keepalive);
                queued_bytes_ += first->data->size();

                for (auto it = std::next(first); it != queue_.end();) {
                    if (it->replace_key == replace_key) {
                        queued_bytes_ -= it->data->size();
                        it = queue_.erase(it);
                    } else {
                        ++it;
                    }
                }
                return;
            }
        }

        if (data.size() > max_queued_bytes_ - queued_bytes_) {
            throw write_queue_full{queued_bytes_};
        }

        auto replacement = std::make_shared<std::vector<std::byte>>(data.begin(), data.end());
        auto owner_keepalive = keepalive;
        queue_.push_back(item{
            std::move(replacement), std::move(keepalive), replace_key});
        queued_bytes_ += data.size();
        if (!writing_) {
            writing_ = true;
            // Keep the owner alive for the complete drain coroutine.  A
            // per-frame keepalive alone can be released immediately after
            // the final write, before drain() performs its final member
            // accesses.
            drain_keepalive_ = std::move(owner_keepalive);
            roboctrl::spawn(drain());
        }
    }

    awaitable<void> drain() {
        auto drain_keepalive = drain_keepalive_;
        try {
            while (!queue_.empty()) {
                auto frame = std::move(queue_.front());
                queue_.pop_front();
                queued_bytes_ -= frame.data->size();
                co_await writer_(std::span<const std::byte>{frame.data->data(), frame.data->size()});
            }
        } catch (const asio::system_error& error) {
            if (error.code() != asio::error::operation_aborted) {
                write_failure_ = std::current_exception();
                logger::instance().log_error(
                    "asynchronous write failed: {}", error.what());
            }
            queue_.clear();
            queued_bytes_ = 0;
        } catch (const std::exception& error) {
            queue_.clear();
            queued_bytes_ = 0;
            write_failure_ = std::current_exception();
            roboctrl::logger::instance().log_error(
                "asynchronous write failed: {}", error.what());
        } catch (...) {
            queue_.clear();
            queued_bytes_ = 0;
            write_failure_ = std::current_exception();
            roboctrl::logger::instance().log_error(
                "asynchronous write failed with an unknown exception");
        }
        writing_ = false;
        drain_keepalive_.reset();
    }

    writer_type writer_;
    struct item {
        std::shared_ptr<std::vector<std::byte>> data;
        std::shared_ptr<void> keepalive;
        std::optional<std::uint64_t> replace_key;
    };

    std::deque<item> queue_;
    std::size_t queued_bytes_{0};
    bool writing_{false};
    std::shared_ptr<void> drain_keepalive_;
    std::exception_ptr write_failure_;

    static constexpr std::size_t max_queued_bytes_ = 64 * 1024;
};

} // namespace roboctrl::io
