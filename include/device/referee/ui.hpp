#pragma once

#include "device/referee/protocol.hpp"
#include <array>
#include <bit>
#include <cmath>
#include <optional>
#include <string_view>
#include <utility>

namespace roboctrl::device::referee_protocol {

enum class graphic_type : std::uint8_t { line, rectangle, circle, ellipse, arc, floating, integer, text };
struct graphic {
    std::array<std::uint8_t, 3> name {};
    std::uint8_t operation {1};
    graphic_type type {graphic_type::line};
    std::uint8_t layer {}, color {2};
    std::uint16_t start_angle {}, end_angle {}, width {2}, x {}, y {}, radius {}, end_x {}, end_y {};
    // Numeric bits are explicit: legacy floating UI used IEEE754 float, integer uses int32.
    // Callers must verify their client protocol revision before using numeric graphics.
    std::optional<std::uint32_t> numeric_bits {};
};

inline std::array<std::byte, 15> encode_graphic(const graphic& item) {
    if (item.operation < 1 || item.operation > 3 || std::uint8_t(item.type) > 7 ||
        item.layer > 9 || item.color > 8 || item.start_angle > 511 || item.end_angle > 511 ||
        item.width > 1023 || item.x > 2047 || item.y > 2047 || item.radius > 1023 ||
        item.end_x > 2047 || item.end_y > 2047)
        throw std::invalid_argument("referee graphic field exceeds wire width/range");
    std::vector<std::byte> result;
    for (auto ch : item.name) result.push_back(std::byte{ch});
    append_u32(result, std::uint32_t(item.operation) | (std::uint32_t(item.type) << 3) |
        (std::uint32_t(item.layer) << 6) | (std::uint32_t(item.color) << 10) |
        (std::uint32_t(item.start_angle) << 14) | (std::uint32_t(item.end_angle) << 23));
    append_u32(result, std::uint32_t(item.width) | (std::uint32_t(item.x) << 10) |
        (std::uint32_t(item.y) << 21));
    append_u32(result, item.numeric_bits.value_or(std::uint32_t(item.radius) |
        (std::uint32_t(item.end_x) << 10) | (std::uint32_t(item.end_y) << 21)));
    std::array<std::byte, 15> out{};
    std::copy(result.begin(), result.end(), out.begin());
    return out;
}

inline std::optional<std::uint16_t> client_id(std::uint16_t robot_id) {
    if ((robot_id >= 1 && robot_id <= 6) || (robot_id >= 101 && robot_id <= 106))
        return robot_id + 0x0100;
    return std::nullopt; // Sentry has no operator client in this source profile.
}
inline std::vector<std::byte> interactive_header(std::uint16_t content_id,
    std::uint16_t sender, std::uint16_t receiver) {
    if (sender == 0 || receiver == 0) throw std::invalid_argument("referee UI IDs must be nonzero");
    std::vector<std::byte> out;
    append_u16(out, content_id);
    append_u16(out, sender);
    append_u16(out, receiver);
    return out;
}
inline std::vector<std::byte> encode_graphics(std::span<const graphic> items,
    std::uint16_t sender, std::uint16_t receiver, std::uint8_t sequence) {
    std::uint16_t content_id{};
    switch (items.size()) {
    case 1: content_id = 0x0101; break;
    case 2: content_id = 0x0102; break;
    case 5: content_id = 0x0103; break;
    case 7: content_id = 0x0104; break;
    default: throw std::invalid_argument("referee graphic count must be 1, 2, 5 or 7");
    }
    auto payload = interactive_header(content_id, sender, receiver);
    for (const auto& item : items) {
        if (item.type == graphic_type::text) throw std::invalid_argument("text uses its own frame");
        auto encoded = encode_graphic(item);
        payload.insert(payload.end(), encoded.begin(), encoded.end());
    }
    return encode_frame(0x0301, payload, sequence);
}
inline std::vector<std::byte> encode_text(graphic item, std::string_view text,
    std::uint16_t sender, std::uint16_t receiver, std::uint8_t sequence) {
    if (text.size() > 30) throw std::invalid_argument("referee UI text exceeds 30 bytes");
    item.type = graphic_type::text;
    item.end_angle = static_cast<std::uint16_t>(text.size());
    auto payload = interactive_header(0x0110, sender, receiver);
    auto encoded = encode_graphic(item);
    payload.insert(payload.end(), encoded.begin(), encoded.end());
    for (char ch : text) payload.push_back(std::byte(static_cast<unsigned char>(ch)));
    payload.resize(51, std::byte{0});
    return encode_frame(0x0301, payload, sequence);
}
inline std::vector<std::byte> encode_delete(std::uint8_t operation, std::uint8_t layer,
    std::uint16_t sender, std::uint16_t receiver, std::uint8_t sequence) {
    if (operation < 1 || operation > 2 || layer > 9)
        throw std::invalid_argument("referee UI delete operation/layer invalid");
    auto payload = interactive_header(0x0100, sender, receiver);
    payload.push_back(std::byte{operation});
    payload.push_back(std::byte{layer});
    return encode_frame(0x0301, payload, sequence);
}

using ui_address = std::pair<std::uint16_t, std::uint16_t>;

/** Track refreshes for a usable sender/receiver, not unrelated referee traffic. */
class ui_refresh_state {
public:
    [[nodiscard]] bool add_required(ui_address address) const {
        return !address_ || *address_ != address || refresh_ == 0;
    }
    void submitted(ui_address address) {
        if (!address_ || *address_ != address) refresh_ = 0;
        address_ = address;
        refresh_ = (refresh_ + 1) % 50;
    }
    void reset() { address_.reset(); refresh_ = 0; }
private:
    std::optional<ui_address> address_;
    unsigned refresh_ {};
};

struct ui_status {
    bool friction_ready {}, auto_aim {}, spinning {}, fire_permitted {};
    float capacitor_percent {};
};
inline std::array<graphic, 5> status_graphics(const ui_status& status, bool add) {
    std::array<graphic, 5> out{};
    const std::array<bool, 4> flags{status.friction_ready, status.auto_aim, status.spinning, status.fire_permitted};
    for (std::size_t i = 0; i < flags.size(); ++i) {
        out[i] = {.name = {'S', 'T', static_cast<std::uint8_t>('0' + i)},
            .operation = static_cast<std::uint8_t>(add ? 1 : 2), .type = graphic_type::circle,
            .layer = 8, .color = static_cast<std::uint8_t>(flags[i] ? 2 : 1), .width = 4,
            .x = static_cast<std::uint16_t>(800 + i * 70), .y = 500, .radius = 16};
    }
    const auto percent = std::isfinite(status.capacitor_percent) ?
        std::clamp(status.capacitor_percent, 0.0f, 100.0f) : 0.0f;
    out[4] = {.name = {'C', 'A', 'P'}, .operation = static_cast<std::uint8_t>(add ? 1 : 2),
        .type = graphic_type::line, .layer = 8, .color = 2, .width = 8, .x = 800, .y = 450,
        .end_x = static_cast<std::uint16_t>(800 + percent * 2), .end_y = 450};
    return out;
}
} // namespace roboctrl::device::referee_protocol
