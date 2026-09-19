#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "device/motor/protocol.hpp"
#include "device/super_cap_protocol.hpp"

namespace {
void require_motor(bool result, const char* message) {
    if (!result) throw std::runtime_error(message);
}
bool close_motor(float a, float b, float epsilon = 1.e-5f) { return std::abs(a - b) <= epsilon; }
}

void test_motor_protocols() {
    using namespace roboctrl::device::motor_protocol;
    const auto positive = encode_m9025_current(0x1234);
    const frame positive_expected{std::byte{0xa0}, {}, {}, {}, std::byte{0x34}, std::byte{0x12}, {}, {}};
    require_motor(positive == positive_expected, "M9025 command layout/current little endian");
    const auto negative = encode_m9025_current(-300);
    require_motor(negative[4] == std::byte{0xd4} && negative[5] == std::byte{0xfe}, "M9025 negative current");
    const frame feedback{std::byte{0xa0}, std::byte{50}, std::byte{0xd4}, std::byte{0xfe},
        std::byte{0xa6}, std::byte{0xff}, std::byte{0x00}, std::byte{0x80}};
    auto parsed = decode_m9025(feedback);
    require_motor(parsed && parsed->current_raw == -300 && parsed->speed_raw == -90
        && parsed->encoder == 32768 && parsed->temperature == 50, "M9025 feedback field offsets and sign");
    require_motor(!decode_m9025(std::span{feedback}.first(7)), "M9025 truncated feedback");
    auto wrong = feedback;
    wrong[0] = std::byte{0xa1};
    require_motor(!decode_m9025(wrong), "M9025 unsupported command echo");
    std::array<std::byte, 9> long_frame{};
    long_frame[0] = std::byte{0xa0};
    require_motor(!decode_m9025(long_frame), "M9025 oversized feedback");

    const auto velocity = encode_j6006_velocity(-1.5f);
    const std::array<std::byte, 4> velocity_expected{{{}, {}, std::byte{0xc0}, std::byte{0xbf}}};
    require_motor(velocity == velocity_expected, "J6006 IEEE754 little endian DLC4");
    const auto enable = encode_j6006_enabled(true);
    const auto disable = encode_j6006_enabled(false);
    for (size_t i = 0; i < 7; ++i)
        require_motor(enable[i] == std::byte{0xff} && disable[i] == std::byte{0xff}, "J6006 special command prefix");
    require_motor(enable[7] == std::byte{0xfc} && disable[7] == std::byte{0xfd}, "J6006 enable disable opcode");
    frame j_feedback{std::byte{0x11}, {}, {}, {}, {}, {}, std::byte{40}, std::byte{45}};
    auto j = decode_j6006(j_feedback, 1, {});
    require_motor(j && j->controller_id == 1 && j->status == 1 && j->position_rad == -12.5f
        && j->velocity_rad_s == -45.f && j->torque_nm == -20.f, "J6006 lower mapping endpoints");
    for (size_t i = 1; i <= 5; ++i) j_feedback[i] = std::byte{0xff};
    j = decode_j6006(j_feedback, 1, {});
    require_motor(j && j->position_raw == 65535 && close_motor(j->position_rad, 12.5f)
        && close_motor(j->velocity_rad_s, 45.f) && close_motor(j->torque_nm, 20.f), "J6006 upper mapping endpoints");
    require_motor(!decode_j6006(j_feedback, 2, {}), "J6006 controller ID mismatch");
    require_motor(!decode_j6006(std::span{j_feedback}.first(7), 1, {}), "J6006 short feedback");
    require_motor(!decode_j6006(long_frame, 1, {}), "J6006 oversized feedback");
    j_feedback[0] = std::byte{0x81};
    j = decode_j6006(j_feedback, 1, {});
    require_motor(j && j->status == 8, "J6006 fault telemetry remains observable");
    require_motor(gated_j6006_velocity(12.f, true, true, 8) == 0.f, "J6006 fault suppresses command");
    require_motor(gated_j6006_velocity(12.f, true, true, 0) == 0.f, "J6006 disabled status suppresses command");
    require_motor(gated_j6006_velocity(12.f, true, false, 1) == 0.f, "J6006 offline suppresses command");
    require_motor(gated_j6006_velocity(12.f, false, true, 1) == 0.f, "J6006 software gate");
    require_motor(gated_j6006_velocity(12.f, true, true, 1) == 12.f, "J6006 active velocity");
    j_feedback[0] = std::byte{0x31};
    require_motor(!decode_j6006(j_feedback, 1, {}), "J6006 reserved status");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    require_motor(!valid_ranges({nan, 45.f, 20.f}), "J6006 invalid feedback range");
    require_motor(encode_j6006_velocity(nan) == std::array<std::byte, 4>{}, "J6006 invalid setpoint becomes zero");
    require_motor(gated_current(1000, .5f, 300, true, true) == 300, "absolute current cap after scale");
    require_motor(gated_current(-1000, .5f, 300, true, true) == -300, "negative current cap");
    require_motor(gated_current(32000, 1.f, 300, true, true) == 300, "new PID output cannot bypass stale allocated cap");
    require_motor(gated_current(1000, 1.f, 300, false, true) == 0, "disabled current");
    require_motor(gated_current(1000, 1.f, 300, true, false) == 0, "offline current");
    require_motor(gated_current(nan, 1.f, 300, true, true) == 0, "nonfinite current");
    require_motor(gated_current(1000, nan, 300, true, true) == 0, "nonfinite output scale");
    require_motor(gated_current(1000, 1.f, -1, true, true) == 0, "invalid current cap");
    require_motor(gated_current(100000, 1.f, std::numeric_limits<float>::infinity(), true, true) == 32767,
        "current conversion saturates instead of wrapping");

    namespace cap = roboctrl::device::super_cap_protocol;
    const frame cap_feedback{std::byte{0}, {}, {}, std::byte{0xc8}, std::byte{0x42}, std::byte{0x2c}, std::byte{1}, std::byte{70}};
    const auto c = cap::decode(cap_feedback);
    require_motor(c && close_motor(c->chassis_power, 100.f) && c->power_limit == 300 && c->energy == 70,
        "supercap float and integer little endian");
    require_motor(!cap::decode(std::span{cap_feedback}.first(7)), "supercap short feedback");
    auto bad_cap = cap_feedback;
    bad_cap[3] = std::byte{0xc0}; bad_cap[4] = std::byte{0x7f};
    require_motor(!cap::decode(bad_cap), "supercap rejects NaN power");
    const auto cap_command = cap::encode(true, 300, 50);
    require_motor(cap_command[0] == std::byte{1} && cap_command[1] == std::byte{0x2c}
        && cap_command[2] == std::byte{1} && cap_command[3] == std::byte{50}, "supercap command layout");
    require_motor(cap::output_enabled(true, true, true, 0), "supercap active gate");
    require_motor(!cap::output_enabled(true, false, true, 0), "supercap stale controller gate");
    require_motor(!cap::output_enabled(true, true, false, 0), "supercap offline gate");
    require_motor(!cap::output_enabled(true, true, true, 1), "supercap error gate");
}

#ifdef ROBOCTRL_MOTOR_PROTOCOL_TEST_MAIN
int main() { test_motor_protocols(); }
#endif
