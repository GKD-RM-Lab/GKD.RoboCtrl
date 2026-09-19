#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace roboctrl::device::motor_protocol {

using frame = std::array<std::byte, 8>;

inline uint8_t octet(std::span<const std::byte> data, size_t index) {
    return std::to_integer<uint8_t>(data[index]);
}

inline uint16_t read_u16_le(std::span<const std::byte> data, size_t index) {
    return static_cast<uint16_t>(octet(data, index) | uint16_t{octet(data, index + 1)} << 8);
}

inline int16_t read_i16_le(std::span<const std::byte> data, size_t index) {
    return std::bit_cast<int16_t>(read_u16_le(data, index));
}

inline int16_t gated_current(float command, float scale, float limit, bool enabled, bool online) {
    if (!enabled || !online || !std::isfinite(command) || !std::isfinite(scale)
        || std::isnan(limit) || limit < 0.f) return 0;
    const float bound = std::min(limit, 32767.f);
    return static_cast<int16_t>(std::clamp(command * std::clamp(scale, 0.f, 1.f), -bound, bound));
}

struct m9025_feedback {
    uint8_t temperature;
    int16_t current_raw;
    int16_t speed_raw;
    uint16_t encoder;
};

inline std::optional<m9025_feedback> decode_m9025(std::span<const std::byte> data) {
    if (data.size() != 8 || octet(data, 0) != 0xa0) return std::nullopt;
    return m9025_feedback{octet(data, 1), read_i16_le(data, 2), read_i16_le(data, 4), read_u16_le(data, 6)};
}

inline frame encode_m9025_current(int16_t current) {
    frame data{};
    const auto raw = std::bit_cast<uint16_t>(current);
    data[0] = std::byte{0xa0};
    data[4] = static_cast<std::byte>(raw & 0xff);
    data[5] = static_cast<std::byte>(raw >> 8);
    return data;
}

struct j6006_ranges {
    float position_max {12.5f};
    float velocity_max {45.f};
    float torque_max {20.f};
};

struct j6006_feedback {
    uint8_t controller_id;
    uint8_t status;
    uint16_t position_raw;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
};

inline bool valid_ranges(const j6006_ranges& ranges) {
    return std::isfinite(ranges.position_max) && ranges.position_max > 0.f
        && std::isfinite(ranges.velocity_max) && ranges.velocity_max > 0.f
        && std::isfinite(ranges.torque_max) && ranges.torque_max > 0.f;
}

inline std::optional<j6006_feedback> decode_j6006(
    std::span<const std::byte> data, uint16_t id, const j6006_ranges& ranges) {
    if (data.size() != 8 || (octet(data, 0) & 0x0f) != (id & 0x0f) || !valid_ranges(ranges))
        return std::nullopt;
    const auto status = static_cast<uint8_t>(octet(data, 0) >> 4);
    if (status > 1 && (status < 8 || status > 14)) return std::nullopt;
    const auto position = static_cast<uint16_t>(uint16_t{octet(data, 1)} << 8 | octet(data, 2));
    const auto velocity = static_cast<uint16_t>(uint16_t{octet(data, 3)} << 4 | octet(data, 4) >> 4);
    const auto torque = static_cast<uint16_t>((octet(data, 4) & 0x0f) << 8 | octet(data, 5));
    return j6006_feedback{
        static_cast<uint8_t>(octet(data, 0) & 0x0f), static_cast<uint8_t>(octet(data, 0) >> 4), position,
        (2.f * static_cast<float>(position) / 65535.f - 1.f) * ranges.position_max,
        (2.f * static_cast<float>(velocity) / 4095.f - 1.f) * ranges.velocity_max,
        (2.f * static_cast<float>(torque) / 4095.f - 1.f) * ranges.torque_max,
        octet(data, 6), octet(data, 7)};
}

inline std::array<std::byte, 4> encode_j6006_velocity(float velocity) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    std::array<std::byte, 4> data{};
    const auto raw = std::bit_cast<uint32_t>(std::isfinite(velocity) ? velocity : 0.f);
    for (size_t i = 0; i < 4; ++i) data[i] = static_cast<std::byte>((raw >> (8 * i)) & 0xff);
    return data;
}

inline frame encode_j6006_enabled(bool enabled) {
    frame data;
    data.fill(std::byte{0xff});
    data[7] = enabled ? std::byte{0xfc} : std::byte{0xfd};
    return data;
}

inline float gated_j6006_velocity(float velocity, bool enabled, bool online, uint8_t status) {
    return enabled && online && status == 1 && std::isfinite(velocity) ? velocity : 0.f;
}

} // namespace roboctrl::device::motor_protocol
