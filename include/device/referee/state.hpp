#pragma once

#include "device/referee/protocol.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <optional>

namespace roboctrl::device::referee_protocol {

using clock = std::chrono::steady_clock;
template<typename T> struct timed_value {
    T value {};
    std::optional<clock::time_point> received_at;
    bool fresh(clock::time_point now, clock::duration timeout) const {
        return received_at && now >= *received_at && now - *received_at <= timeout;
    }
    void set(T next, clock::time_point now) { value = next; received_at = now; }
};
struct game_status {
    std::uint8_t game_type {}, progress {};
    std::uint16_t remaining_seconds {};
    std::uint64_t timestamp {};
};
struct robot_status {
    std::uint8_t robot_id {}, level {};
    std::uint16_t hp {}, max_hp {}, cooling_rate {}, cooling_limit {}, chassis_power_limit {};
    bool gimbal_power {}, chassis_power {}, shooter_power {};
};
struct power_heat {
    std::uint16_t buffer_energy {}, heat_17_1 {}, heat_17_2 {}, heat_42 {};
};
struct bullet_allowance { std::uint16_t bullets_17 {}, bullets_42 {}, coins {}; };
struct robot_position { float x {}, y {}, yaw {}; };
struct shot_data { std::uint8_t caliber {}, shooter_id {}, frequency {}; float speed {}; };
struct referee_warning { std::uint8_t level {}, robot_id {}, count {}; };

/** Source profile: GKD_Control/include/device/referee/protocol.hpp (unversioned).
 * Known commands require their exact source layout; no inferred year upgrades.
 */
struct state {
    timed_value<game_status> game;
    timed_value<robot_status> robot;
    timed_value<power_heat> power;
    timed_value<bullet_allowance> ammunition;
    timed_value<robot_position> position;
    timed_value<shot_data> shot;
    timed_value<referee_warning> warning;
    timed_value<std::uint8_t> winner;
    timed_value<std::array<std::uint16_t, 16>> robot_hp;

    bool accept(const frame& input, clock::time_point now) {
        const bytes p{input.payload};
        switch (input.command) {
        case 0x0001:
            if (p.size() != 11 || (u8(p[0]) >> 4) > 5) return false;
            game.set({static_cast<std::uint8_t>(u8(p[0]) & 0x0f),
                static_cast<std::uint8_t>(u8(p[0]) >> 4), u16(p, 1),
                u32(p, 3) | (std::uint64_t{u32(p, 7)} << 32)}, now);
            return true;
        case 0x0002:
            if (p.size() != 1 || u8(p[0]) > 2) return false;
            winner.set(u8(p[0]), now);
            return true;
        case 0x0003: {
            if (p.size() != 32) return false;
            std::array<std::uint16_t, 16> hp{};
            for (std::size_t i = 0; i < hp.size(); ++i) hp[i] = u16(p, i * 2);
            robot_hp.set(hp, now);
            return true;
        }
        case 0x0104:
            if (p.size() != 3) return false;
            warning.set({u8(p[0]), u8(p[1]), u8(p[2])}, now);
            return true;
        case 0x0201: {
            if (p.size() != 13 || u8(p[0]) == 0 || (u8(p[12]) & 0xf8)) return false;
            robot.set({u8(p[0]), u8(p[1]), u16(p, 2), u16(p, 4), u16(p, 6),
                u16(p, 8), u16(p, 10), bool(u8(p[12]) & 1), bool(u8(p[12]) & 2),
                bool(u8(p[12]) & 4)}, now);
            return true;
        }
        case 0x0202:
            if (p.size() != 16) return false;
            power.set({u16(p, 8), u16(p, 10), u16(p, 12), u16(p, 14)}, now);
            return true;
        case 0x0203: {
            if (p.size() != 12) return false;
            const robot_position next{std::bit_cast<float>(u32(p, 0)),
                std::bit_cast<float>(u32(p, 4)), std::bit_cast<float>(u32(p, 8))};
            if (!std::isfinite(next.x) || !std::isfinite(next.y) || !std::isfinite(next.yaw)) return false;
            position.set(next, now);
            return true;
        }
        case 0x0207: {
            if (p.size() != 7) return false;
            const auto speed = std::bit_cast<float>(u32(p, 3));
            if (!std::isfinite(speed) || speed < 0 || u8(p[0]) < 1 || u8(p[0]) > 2) return false;
            shot.set({u8(p[0]), u8(p[1]), u8(p[2]), speed}, now);
            return true;
        }
        case 0x0208:
            if (p.size() != 6) return false;
            ammunition.set({u16(p, 0), u16(p, 2), u16(p, 4)}, now);
            return true;
        default:
            return false;
        }
    }
};
} // namespace roboctrl::device::referee_protocol
