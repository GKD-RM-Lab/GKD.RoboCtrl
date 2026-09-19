#include "device/referee/state.hpp"
#include "device/referee/ui.hpp"
#include <array>
#include <stdexcept>
#include <string_view>

using namespace roboctrl::device::referee_protocol;
using namespace std::chrono_literals;

namespace {
void require(bool condition, std::string_view message) {
    if (!condition) throw std::runtime_error(std::string{message});
}
template<typename Fn> void rejects(Fn fn) {
    bool rejected = false;
    try { fn(); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid referee/UI input accepted");
}
}

void run_referee_protocol_tests() {
    const std::array<std::byte, 9> text{std::byte{'1'}, std::byte{'2'}, std::byte{'3'},
        std::byte{'4'}, std::byte{'5'}, std::byte{'6'}, std::byte{'7'}, std::byte{'8'}, std::byte{'9'}};
    // Independently produced from the old referee_base lookup tables, not this encoder.
    require(crc8(text) == 0x0b && crc16(text) == 0x6f91, "legacy CRC check vector mismatch");
    const std::array<std::byte, 15> golden{std::byte{0xa5}, std::byte{6}, std::byte{0},
        std::byte{7}, std::byte{0x91}, std::byte{8}, std::byte{2}, std::byte{30}, std::byte{0},
        std::byte{4}, std::byte{0}, std::byte{9}, std::byte{0}, std::byte{0x68}, std::byte{0x21}};
    const bytes payload{golden.data() + 7, 6};
    require(encode_frame(0x0208, payload, 7) == std::vector<std::byte>(golden.begin(), golden.end()),
        "referee encoder differs from legacy independent frame");
    for (std::size_t split = 0; split <= golden.size(); ++split) {
        stream_parser parser;
        unsigned calls = 0;
        parser.feed(bytes{golden}.first(split), [&](const frame&) { ++calls; });
        parser.feed(bytes{golden}.subspan(split), [&](const frame& frame) {
            require(frame.command == 0x0208 && frame.sequence == 7 && frame.payload.size() == 6,
                "referee frame decode mismatch");
            ++calls;
        });
        require(calls == 1 && parser.buffered_size() == 0, "fragmentation lost or duplicated frame");
    }
    stream_parser parser;
    std::vector<std::byte> corrupt(golden.begin(), golden.end());
    corrupt[8] ^= std::byte{1};
    unsigned calls = 0;
    parser.feed(corrupt, [&](const frame&) { ++calls; });
    parser.feed(golden, [&](const frame&) { ++calls; });
    require(calls == 1 && parser.rejected_frames() > 0, "CRC failure failed to resynchronize");
    std::vector<std::byte> noise(10000, std::byte{0xaa});
    parser.feed(noise, [&](const frame&) { ++calls; });
    require(parser.buffered_size() == 0, "noise retained without bound");
    std::vector<std::byte> oversized{std::byte{0xa5}, std::byte{1}, std::byte{1}, std::byte{1}};
    oversized.push_back(std::byte{crc8(oversized)});
    oversized.insert(oversized.end(), golden.begin(), golden.end());
    parser.feed(oversized, [&](const frame&) { ++calls; });
    require(calls == 2, "oversized frame length failed to resynchronize");
    auto bad_header = golden;
    bad_header[4] ^= std::byte{1};
    parser.feed(bad_header, [&](const frame&) { ++calls; });
    require(calls == 2, "header CRC not checked");

    state telemetry;
    const auto now = clock::time_point{} + 1s;
    require(!telemetry.ammunition.fresh(now, 100ms), "unseen referee data considered fresh");
    const frame ammo{0x0208, 7, {payload.begin(), payload.end()}};
    require(telemetry.accept(ammo, now), "ammo command rejected");
    require(telemetry.ammunition.value.bullets_17 == 30 &&
        telemetry.ammunition.value.bullets_42 == 4 && telemetry.ammunition.value.coins == 9,
        "caliber or endian decode mismatch");
    require(telemetry.ammunition.fresh(now + 100ms, 100ms) &&
        !telemetry.ammunition.fresh(now + 101ms, 100ms) && !telemetry.ammunition.fresh(now - 1ns, 100ms),
        "referee freshness boundary mismatch");
    auto short_ammo = ammo;
    short_ammo.payload.pop_back();
    require(!telemetry.accept(short_ammo, now + 1s) && *telemetry.ammunition.received_at == now,
        "invalid length refreshed ammo state");
    require(!telemetry.accept({0xffff, 0, {}}, now), "unknown command decoded as known state");
    std::vector<std::byte> status(13);
    status[0] = std::byte{101}; status[2] = std::byte{100}; status[10] = std::byte{80}; status[12] = std::byte{7};
    require(telemetry.accept({0x0201, 0, status}, now), "robot status decode failed");
    require(telemetry.robot.value.robot_id == 101 && telemetry.robot.value.hp == 100 &&
        telemetry.robot.value.chassis_power_limit == 80 && telemetry.robot.value.shooter_power,
        "robot status bits/fields mismatch");
    std::vector<std::byte> game(11); game[0] = std::byte{0x41};
    require(telemetry.accept({0x0001, 0, game}, now) && telemetry.game.value.progress == 4,
        "game status nibble mismatch");
    game[0] = std::byte{0xf1};
    require(!telemetry.accept({0x0001, 0, game}, now), "invalid match state accepted");
    std::vector<std::byte> position(12);
    position[2] = std::byte{0x80}; position[3] = std::byte{0x7f};
    require(!telemetry.accept({0x0203, 0, position}, now), "infinite referee position accepted");

    ui_refresh_state refresh;
    const ui_address red_client{1, 0x101}, blue_client{101, 0x165};
    // Game packets can arrive for many cycles before a usable robot identity.
    for (unsigned i = 0; i < 70; ++i) refresh.reset();
    require(refresh.add_required(red_client), "missing identity consumed initial UI Add");
    require(refresh.add_required(red_client), "planning without submission consumed UI Add");
    refresh.submitted(red_client);
    require(!refresh.add_required(red_client), "submitted UI Add was not followed by Modify");
    // Status can expire while other referee packets keep the connection online.
    refresh.reset();
    require(refresh.add_required(red_client), "identity recovery failed to recreate UI graphics");
    refresh.submitted(red_client);
    require(refresh.add_required(blue_client), "robot/client identity change failed to request Add");
    refresh.submitted(blue_client);
    for (unsigned i = 1; i < 50; ++i) {
        require(!refresh.add_required(blue_client), "premature periodic UI Add");
        refresh.submitted(blue_client);
    }
    require(refresh.add_required(blue_client), "periodic UI recovery Add missing");

    const graphic line{.name = {'A', 'B', 'C'}, .operation = 2, .type = graphic_type::line,
        .layer = 1, .color = 2, .width = 3, .x = 100, .y = 200, .end_x = 300, .end_y = 400};
    const auto encoded = encode_graphic(line);
    require(u32(encoded, 3) == 0x842 && u32(encoded, 7) == (3u | (100u << 10) | (200u << 21)) &&
        u32(encoded, 11) == ((300u << 10) | (400u << 21)), "UI explicit bit packing differs");
    require(client_id(1) == 0x101 && client_id(101) == 0x165 && !client_id(7), "UI client ID mapping differs");
    std::array<graphic, 1> graphics{line};
    auto graphic_frame = encode_graphics(graphics, 101, 0x165, 8);
    unsigned ui_calls = 0;
    parser.feed(graphic_frame, [&](const frame& frame) {
        require(frame.command == 0x0301 && frame.payload.size() == 21 && u16(frame.payload, 0) == 0x0101 &&
            u16(frame.payload, 2) == 101 && u16(frame.payload, 4) == 0x165, "UI interaction header differs");
        ++ui_calls;
    });
    parser.feed(encode_text(line, "READY", 101, 0x165, 9), [&](const frame& frame) {
        require(frame.payload.size() == 51 && frame.payload.back() == std::byte{0}, "UI string not padded");
        ++ui_calls;
    });
    require(ui_calls == 2, "UI frames failed CRC parser");
    auto invalid_graphic = line; invalid_graphic.x = 2048;
    rejects([&] { (void)encode_graphic(invalid_graphic); });
    rejects([&] { (void)encode_graphics(std::span<const graphic>{}, 1, 0x101, 0); });
    rejects([&] { (void)encode_text(line, std::string(31, 'x'), 1, 0x101, 0); });
    rejects([&] { (void)encode_delete(0, 1, 1, 0x101, 0); });
    require(status_graphics({.capacitor_percent = 1000}, true)[4].end_x == 1000,
        "UI energy percentage did not clamp");
}

#ifdef ROBOCTRL_REFEREE_TEST_MAIN
int main() { run_referee_protocol_tests(); }
#endif
