#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace roboctrl::device::referee_protocol {

using bytes = std::span<const std::byte>;
inline constexpr std::size_t max_payload_size = 256;
inline std::uint8_t u8(std::byte value) { return std::to_integer<std::uint8_t>(value); }
inline std::uint16_t u16(bytes value, std::size_t offset) {
    return u8(value[offset]) | (std::uint16_t{u8(value[offset + 1])} << 8);
}
inline std::uint32_t u32(bytes value, std::size_t offset) {
    return u16(value, offset) | (std::uint32_t{u16(value, offset + 2)} << 16);
}
inline void append_u16(std::vector<std::byte>& out, std::uint16_t value) {
    out.push_back(std::byte(value & 0xff));
    out.push_back(std::byte(value >> 8));
}
inline void append_u32(std::vector<std::byte>& out, std::uint32_t value) {
    append_u16(out, value & 0xffff);
    append_u16(out, value >> 16);
}

// Legacy RM referee wire CRCs: reflected x^8+x^5+x^4+1 and
// x^16+x^12+x^5+1, initial all ones, no final XOR.
inline std::uint8_t crc8(bytes data, std::uint8_t crc = 0xff) {
    for (const auto value : data) {
        crc ^= u8(value);
        for (int i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8c : crc >> 1;
    }
    return crc;
}
inline std::uint16_t crc16(bytes data, std::uint16_t crc = 0xffff) {
    for (const auto value : data) {
        crc ^= u8(value);
        for (int i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    return crc;
}

struct frame {
    std::uint16_t command {};
    std::uint8_t sequence {};
    std::vector<std::byte> payload;
};

inline std::vector<std::byte> encode_frame(std::uint16_t command, bytes payload, std::uint8_t sequence) {
    if (payload.size() > max_payload_size)
        throw std::invalid_argument("referee payload exceeds legacy profile maximum");
    std::vector<std::byte> out{std::byte{0xa5}};
    append_u16(out, static_cast<std::uint16_t>(payload.size()));
    out.push_back(std::byte{sequence});
    out.push_back(std::byte{crc8(out)});
    append_u16(out, command);
    out.insert(out.end(), payload.begin(), payload.end());
    append_u16(out, crc16(out));
    return out;
}

/** Incremental bounded parser. Payload is owned and survives the next feed(). */
class stream_parser {
public:
    template<typename Fn>
    void feed(bytes input, Fn&& on_frame) {
        for (const auto value : input) {
            buffer_.push_back(value);
            drain(on_frame);
        }
    }
    [[nodiscard]] std::size_t buffered_size() const { return buffer_.size(); }
    [[nodiscard]] std::size_t rejected_frames() const { return rejected_frames_; }
private:
    template<typename Fn>
    void drain(Fn& on_frame) {
        while (!buffer_.empty()) {
            if (buffer_.front() != std::byte{0xa5}) {
                buffer_.erase(buffer_.begin());
                continue;
            }
            if (buffer_.size() < 5) return;
            const bytes data{buffer_};
            const auto length = u16(data, 1);
            if (crc8(data.first(4)) != u8(data[4]) || length > max_payload_size) {
                ++rejected_frames_;
                buffer_.erase(buffer_.begin());
                continue;
            }
            const std::size_t total = length + 9;
            if (buffer_.size() < total) return;
            if (crc16(data.first(total - 2)) != u16(data, total - 2)) {
                ++rejected_frames_;
                buffer_.erase(buffer_.begin());
                continue;
            }
            frame parsed{u16(data, 5), u8(data[3]), {data.begin() + 7, data.begin() + 7 + length}};
            buffer_.erase(buffer_.begin(), buffer_.begin() + total);
            on_frame(parsed);
        }
    }
    std::vector<std::byte> buffer_;
    std::size_t rejected_frames_ {};
};
} // namespace roboctrl::device::referee_protocol
