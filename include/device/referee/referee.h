#pragma once

#include "core/async.hpp"
#include "core/logger.h"
#include "device/referee/state.hpp"
#include "device/referee/ui.hpp"
#include "utils/singleton.hpp"
#include <chrono>
#include <functional>
#include <string>

namespace roboctrl::io { class serial; }
namespace roboctrl::device {

/** RM source-profile referee device; raw serial transport, no control policy. */
class referee : public utils::singleton_base<referee>, public logable<referee> {
public:
    struct info_type {
        using owner_type = referee;
        std::string serial_name {"referee"};
        std::chrono::steady_clock::duration offline_timeout {std::chrono::seconds{1}};
        bool ui_enabled {false};
        std::chrono::steady_clock::duration ui_period {std::chrono::milliseconds{100}};
        std::uint16_t ui_receiver_id {0}; // 0 = derive operator client where defined.
    };
    bool init(const info_type& info);
    void connect();
    void start();
    awaitable<void> task();
    [[nodiscard]] bool configured() const { return configured_; }
    [[nodiscard]] bool offline() const;
    [[nodiscard]] auto timeout() const { return info_.offline_timeout; }
    [[nodiscard]] const referee_protocol::state& data() const { return state_; }
    [[nodiscard]] std::string desc() const { return "referee"; }
    void set_ui_status(referee_protocol::ui_status status) { ui_status_ = status; }
    /** All CRC-validated frames can be observed, including unsupported command IDs. */
    void on_frame(std::function<void(const referee_protocol::frame&)> callback) { frame_callback_ = std::move(callback); }
    /** Feed bytes without opening hardware; used by the raw serial callback. */
    void receive(referee_protocol::bytes data);
    awaitable<void> send_graphics(std::span<const referee_protocol::graphic> items);
    awaitable<void> send_text(referee_protocol::graphic item, std::string_view text);
    awaitable<void> delete_graphics(std::uint8_t operation, std::uint8_t layer);
private:
    std::optional<std::pair<std::uint16_t, std::uint16_t>> ui_addresses() const;
    info_type info_;
    io::serial* serial_ {nullptr};
    referee_protocol::stream_parser parser_;
    referee_protocol::state state_;
    referee_protocol::ui_status ui_status_;
    std::optional<referee_protocol::clock::time_point> received_at_;
    std::function<void(const referee_protocol::frame&)> frame_callback_;
    std::uint8_t sequence_ {};
    bool configured_ {}, connected_ {}, started_ {};
};
static_assert(utils::singleton<referee>);
} // namespace roboctrl::device
