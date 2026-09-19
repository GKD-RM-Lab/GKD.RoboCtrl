#pragma once

#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace roboctrl::device {

// The legacy Linux packed structure included a four-byte ROBOT_MODE enum.
// compact_10 is an explicit peer compatibility setting, never auto-detected.
enum class vision_wire_format { legacy_14, compact_10 };

struct aim_target {
    float yaw{};                 // absolute radians, peer/IMU coordinate convention
    float pitch{};
    bool fire{};
    std::uint32_t legacy_mode{}; // informational only; cannot arm the robot
};

struct navigation_command {
    float vx{}; // m/s in the configured navigation/control coordinate frame
    float vy{};
};

namespace network_protocol {
using bytes = std::span<const std::byte>;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);

inline std::uint32_t read_u32(bytes data, std::size_t offset) {
    std::uint32_t result{};
    for (std::size_t i = 0; i < 4; ++i) {
        result |= std::uint32_t(std::to_integer<unsigned char>(data[offset + i])) << (8 * i);
    }
    return result;
}

inline float read_float(bytes data, std::size_t offset) {
    return std::bit_cast<float>(read_u32(data, offset));
}

inline void write_le(std::span<std::byte> data, std::size_t offset,
                     std::uint64_t value, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        data[offset + i] = std::byte((value >> (8 * i)) & 0xff);
    }
}

inline void write_float(std::span<std::byte> data, std::size_t offset, float value) {
    write_le(data, offset, std::bit_cast<std::uint32_t>(value), 4);
}

inline std::optional<aim_target> decode_aim(bytes data, std::uint8_t header,
                                          vision_wire_format format) {
    if (format != vision_wire_format::legacy_14 && format != vision_wire_format::compact_10) {
        return std::nullopt;
    }
    const auto length = format == vision_wire_format::legacy_14 ? 14u : 10u;
    if (data.size() != length || data[0] != std::byte{header} || header == 0x37 ||
        std::to_integer<unsigned char>(data[9]) > 1) {
        return std::nullopt;
    }
    aim_target result{read_float(data, 1), read_float(data, 5), data[9] == std::byte{1}, 0};
    if (!std::isfinite(result.yaw) || !std::isfinite(result.pitch)) {
        return std::nullopt;
    }
    if (format == vision_wire_format::legacy_14) {
        result.legacy_mode = read_u32(data, 10);
        if (result.legacy_mode > 5) {
            return std::nullopt;
        }
    }
    return result;
}

inline std::optional<navigation_command> decode_navigation(bytes data) {
    if (data.size() != 9 || data[0] != std::byte{0x37}) {
        return std::nullopt;
    }
    navigation_command result{read_float(data, 1), read_float(data, 5)};
    if (!std::isfinite(result.vx) || !std::isfinite(result.vy)) {
        return std::nullopt;
    }
    return result;
}

inline std::array<std::byte, 10> encode_posture(std::uint8_t header,
                                             float yaw, float pitch, bool red) {
    if (header == 0x37 || !std::isfinite(yaw) || !std::isfinite(pitch)) {
        throw std::invalid_argument("invalid vision posture");
    }
    std::array<std::byte, 10> data{};
    data[0] = std::byte{header};
    write_float(data, 1, yaw);
    write_float(data, 5, pitch);
    data[9] = std::byte{red};
    return data;
}

inline std::array<std::byte, 10> encode_navigation(float yaw, float hp_fraction,
                                                bool match_started) {
    if (!std::isfinite(yaw) || !std::isfinite(hp_fraction) || hp_fraction < 0 ||
        hp_fraction > 1) {
        throw std::invalid_argument("invalid navigation telemetry");
    }
    std::array<std::byte, 10> data{};
    data[0] = std::byte{0x37};
    write_float(data, 1, yaw);
    write_float(data, 5, hp_fraction);
    data[9] = std::byte{match_started};
    return data;
}

// Local receive time is used: the legacy wire contains no sequence or timestamp.
template<typename T>
class fresh_value {
public:
    using clock = std::chrono::steady_clock;
    void update(T value, clock::time_point now = clock::now()) {
        value_ = value;
        received_ = now;
    }
    std::optional<T> get(clock::duration timeout, clock::time_point now = clock::now()) const {
        if (!value_ || timeout <= clock::duration::zero() || now < received_ ||
            now - received_ >= timeout) {
            return std::nullopt;
        }
        return value_;
    }
    void reset() { value_.reset(); }
private:
    std::optional<T> value_;
    clock::time_point received_{};
};

inline std::uint32_t log_name_id(std::string_view name) {
    std::uint32_t value = 2166136261u;
    for (const unsigned char c : name) {
        value = (value ^ c) * 16777619u;
    }
    return value;
}

inline std::vector<std::byte> encode_log_name(std::string_view name) {
    if (name.empty() || name.size() > 255) {
        throw std::invalid_argument("remote log name must contain 1..255 bytes");
    }
    std::vector<std::byte> data(8 + name.size());
    write_le(data, 0, data.size(), 2);
    data[2] = std::byte{0};
    write_le(data, 3, log_name_id(name), 4);
    data[7] = std::byte(name.size());
    for (std::size_t i = 0; i < name.size(); ++i) data[8 + i] = std::byte(name[i]);
    return data;
}

inline std::array<std::byte, 15> encode_log_value(std::uint32_t id, double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite remote log value");
    std::array<std::byte, 15> data{};
    write_le(data, 0, data.size(), 2);
    data[2] = std::byte{1};
    write_le(data, 3, id, 4);
    write_le(data, 7, std::bit_cast<std::uint64_t>(value), 8);
    return data;
}

inline std::vector<std::byte> encode_log_text(std::string_view text, bool message_box = false) {
    // Keep each complete datagram below IPv4's maximum UDP payload.
    if (text.size() > 65502) throw std::invalid_argument("remote log text too long");
    std::vector<std::byte> data(5 + text.size());
    write_le(data, 0, data.size(), 2);
    data[2] = std::byte(message_box ? 3 : 2);
    write_le(data, 3, text.size(), 2);
    for (std::size_t i = 0; i < text.size(); ++i) data[5 + i] = std::byte(text[i]);
    return data;
}
} // namespace network_protocol
} // namespace roboctrl::device
