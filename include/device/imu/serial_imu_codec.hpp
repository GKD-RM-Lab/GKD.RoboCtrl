#pragma once

#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>

#include "device/imu/base.hpp"

namespace roboctrl::device {

/** Legacy key-1 payload: six IEEE-754 float32 little-endian values, 24 bytes. */
struct serial_imu_packet {
    float yaw, pitch, roll, yaw_v, pitch_v, roll_v;
};

inline std::optional<serial_imu_packet> decode_serial_imu(std::span<const std::byte> bytes) {
    if (bytes.size() != 24) return std::nullopt;
    float values[6]{};
    for (std::size_t i = 0; i < 6; ++i) {
        std::uint32_t bits{};
        for (std::size_t j = 0; j < 4; ++j)
            bits |= std::uint32_t(std::to_integer<unsigned char>(bytes[4 * i + j])) << (8 * j);
        values[i] = std::bit_cast<float>(bits);
        if (!std::isfinite(values[i])) return std::nullopt;
    }
    return serial_imu_packet{values[0], values[1], values[2], values[3], values[4], values[5]};
}

struct serial_imu_sample {
    euler_angle angle;
    three_axis gyro;
};

/** Validate the converted physical snapshot before publishing a heartbeat. */
inline std::optional<serial_imu_sample> convert_serial_imu(
        const serial_imu_packet& packet, three_axis angle_sign,
        three_axis rate_sign, float gyro_scale) {
    constexpr float radians_per_degree = Pi_f / 180.0f;
    const float roll = packet.roll * radians_per_degree * angle_sign.x;
    const float pitch = packet.pitch * radians_per_degree * angle_sign.y;
    const float yaw = packet.yaw * radians_per_degree * angle_sign.z;
    const three_axis gyro{
        packet.roll_v * gyro_scale * rate_sign.x,
        packet.pitch_v * gyro_scale * rate_sign.y,
        packet.yaw_v * gyro_scale * rate_sign.z};
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw) ||
        !std::isfinite(gyro.x) || !std::isfinite(gyro.y) || !std::isfinite(gyro.z))
        return std::nullopt;
    return serial_imu_sample{
        .angle = {utils::rad_format(roll), utils::rad_format(pitch), utils::rad_format(yaw)},
        .gyro = gyro};
}

}
