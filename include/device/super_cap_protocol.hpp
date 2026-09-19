#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

namespace roboctrl::device::super_cap_protocol {

struct feedback {
    uint8_t error;
    float chassis_power;
    uint16_t power_limit;
    uint8_t energy;
};

inline std::optional<feedback> decode(std::span<const std::byte> data) {
    if (data.size() != 8) return std::nullopt;
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    uint32_t raw = 0;
    for (size_t i = 0; i < 4; ++i) raw |= uint32_t{std::to_integer<uint8_t>(data[i + 1])} << (8 * i);
    const float power = std::bit_cast<float>(raw);
    if (!std::isfinite(power)) return std::nullopt;
    return feedback{std::to_integer<uint8_t>(data[0]), power,
        static_cast<uint16_t>(std::to_integer<uint8_t>(data[5]) | uint16_t{std::to_integer<uint8_t>(data[6])} << 8),
        std::to_integer<uint8_t>(data[7])};
}

inline std::array<std::byte, 8> encode(bool enabled, uint16_t power_limit, uint16_t buffer_target) {
    return {static_cast<std::byte>(enabled), static_cast<std::byte>(power_limit & 0xff),
        static_cast<std::byte>(power_limit >> 8), static_cast<std::byte>(buffer_target & 0xff),
        static_cast<std::byte>(buffer_target >> 8), std::byte{0}, std::byte{0}, std::byte{0}};
}

inline bool output_enabled(bool requested, bool command_fresh, bool online, uint8_t error) {
    return requested && command_fresh && online && error == 0;
}

} // namespace roboctrl::device::super_cap_protocol
