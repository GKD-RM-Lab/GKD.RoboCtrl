#include "device/network_protocol.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace {
void require_network(bool value) {
    if (!value) throw std::runtime_error("network protocol test failed");
}
template<typename Fn>
void require_invalid(Fn fn) {
    try { fn(); } catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("expected invalid network input");
}
} // namespace

void test_network_protocols() {
    using namespace roboctrl::device;
    using namespace roboctrl::device::network_protocol;
    using namespace std::chrono_literals;
    const std::array raw{
        std::byte{0x6a}, std::byte{0}, std::byte{0}, std::byte{0x80}, std::byte{0x3f},
        std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0xbf}, std::byte{1},
        std::byte{2}, std::byte{0}, std::byte{0}, std::byte{0}};
    auto target = decode_aim(raw, 0x6a, vision_wire_format::legacy_14);
    require_network(target && target->yaw == 1.0f && target->pitch == -0.5f &&
                    target->fire && target->legacy_mode == 2);
    require_network(!decode_aim(raw, 0x6b, vision_wire_format::legacy_14));
    require_network(!decode_aim(raw, 0x6a, vision_wire_format::compact_10));
    require_network(!decode_aim(raw, 0x6a, static_cast<vision_wire_format>(99)));
    for (std::size_t length = 0; length < raw.size(); ++length) {
        require_network(!decode_aim(bytes{raw}.first(length), 0x6a, vision_wire_format::legacy_14));
    }
    auto compact = bytes{raw}.first(10);
    require_network(bool(decode_aim(compact, 0x6a, vision_wire_format::compact_10)));
    auto bad = raw;
    bad[9] = std::byte{2};
    require_network(!decode_aim(bad, 0x6a, vision_wire_format::legacy_14));
    bad = raw;
    bad[10] = std::byte{6};
    require_network(!decode_aim(bad, 0x6a, vision_wire_format::legacy_14));
    bad = raw;
    write_float(bad, 1, std::numeric_limits<float>::quiet_NaN());
    require_network(!decode_aim(bad, 0x6a, vision_wire_format::legacy_14));
    write_float(bad, 1, std::numeric_limits<float>::infinity());
    require_network(!decode_aim(bad, 0x6a, vision_wire_format::legacy_14));
    std::vector<std::byte> long_frame(raw.begin(), raw.end());
    long_frame.push_back(std::byte{0});
    require_network(!decode_aim(long_frame, 0x6a, vision_wire_format::legacy_14));

    const auto posture = encode_posture(0x6a, 1.0f, -0.5f, true);
    require_network(std::equal(posture.begin(), posture.end(), raw.begin()));
    const auto nav_status = encode_navigation(1.0f, 0.5f, true);
    require_network(nav_status[0] == std::byte{0x37} && nav_status[9] == std::byte{1});
    auto nav_packet = std::array<std::byte, 9>{};
    nav_packet[0] = std::byte{0x37};
    write_float(nav_packet, 1, 1.5f);
    write_float(nav_packet, 5, -2.0f);
    const auto nav = decode_navigation(nav_packet);
    require_network(nav && nav->vx == 1.5f && nav->vy == -2.0f);
    require_network(!decode_navigation(nav_status));
    for (std::size_t length = 0; length < nav_packet.size(); ++length) {
        require_network(!decode_navigation(bytes{nav_packet}.first(length)));
    }
    nav_packet[0] = std::byte{0x6a};
    require_network(!decode_navigation(nav_packet));
    nav_packet[0] = std::byte{0x37};
    write_float(nav_packet, 5, std::numeric_limits<float>::quiet_NaN());
    require_network(!decode_navigation(nav_packet));
    require_invalid([] { encode_navigation(0, -0.01f, false); });
    require_invalid([] { encode_navigation(0, 1.01f, false); });
    require_invalid([] { encode_posture(0x37, 0, 0, false); });

    fresh_value<aim_target> cache;
    const auto time = fresh_value<aim_target>::clock::time_point{} + 1s;
    require_network(!cache.get(100ms, time));
    cache.update(*target, time);
    require_network(bool(cache.get(100ms, time + 99ms)));
    require_network(!cache.get(100ms, time + 100ms));
    require_network(!cache.get(100ms, time - 1ms));
    require_network(!cache.get(0ms, time));
    cache.reset();
    require_network(!cache.get(100ms, time));

    // FNV1a known vector + exact little-endian f64 frame.
    require_network(log_name_id("hello") == 0x4f9f2cabu);
    const auto name = encode_log_name("hello");
    require_network(name.size() == 13 && name[0] == std::byte{13} &&
                    name[2] == std::byte{0} && read_u32(name, 3) == 0x4f9f2cabu &&
                    name[7] == std::byte{5} && name[8] == std::byte{'h'});
    const auto value = encode_log_value(0x01020304, 1.0);
    const std::array expected{
        std::byte{15}, std::byte{0}, std::byte{1}, std::byte{4}, std::byte{3},
        std::byte{2}, std::byte{1}, std::byte{0}, std::byte{0}, std::byte{0},
        std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0xf0}, std::byte{0x3f}};
    require_network(value == expected);
    const auto text = encode_log_text("ok");
    require_network(text.size() == 7 && text[0] == std::byte{7} && text[2] == std::byte{2} &&
                    text[3] == std::byte{2} && text[5] == std::byte{'o'});
    require_network(encode_log_text("ok", true)[2] == std::byte{3});
    require_invalid([] { encode_log_name(""); });
    require_invalid([] { encode_log_name(std::string(256, 'a')); });
    require_invalid([] { encode_log_value(0, std::numeric_limits<double>::infinity()); });
    require_invalid([] { encode_log_text(std::string(65503, 'a')); });
}

#ifdef ROBOCTRL_NETWORK_PROTOCOL_TEST_MAIN
int main() { test_network_protocols(); }
#endif
