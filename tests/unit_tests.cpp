#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <stdexcept>
#include <string>

#include <asio.hpp>

#include "config/config.hpp"
#include "config/validate.hpp"
#include "ctrl/chassis_kinematics.hpp"
#include "device/motor/ref.hpp"
#include "io/base.hpp"

using namespace std::chrono_literals;
using roboctrl::awaitable;
using roboctrl::device::motor_base;
using roboctrl::device::motor_ref;

namespace {

template<int Kind>
class fake_motor : public motor_base {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = fake_motor;

        std::string name;
        std::string key() const { return name; }
    };

    explicit fake_motor(const info_type&) : motor_base{0ns, 0.1f} {}

    awaitable<void> set(float value) {
        last_set = value;
        co_return;
    }

    awaitable<void> enable() {
        enabled = true;
        co_return;
    }

    awaitable<void> task() { co_return; }

    void set_measurements(float angle, float speed, float torque) {
        angle_ = angle;
        angle_speed_ = speed;
        torque_ = torque;
    }

    float last_set {0.0f};
    bool enabled {false};
};

using fake_motor_a = fake_motor<1>;
using fake_motor_b = fake_motor<2>;

static_assert(roboctrl::device::motor<fake_motor_a>);
static_assert(roboctrl::device::motor<fake_motor_b>);
static_assert(sizeof(motor_ref) <= sizeof(void*) * 3);

void test_motor_ref_type_erasure() {
    fake_motor_a a{{"a"}};
    fake_motor_b b{{"b"}};
    a.set_measurements(1.0f, 2.0f, 3.0f);
    b.set_measurements(4.0f, 5.0f, 6.0f);

    motor_ref a_ref{a};
    motor_ref b_ref{b};
    std::array<motor_ref, 2> motors{a_ref, b_ref};

    asio::io_context context;
    auto done = asio::co_spawn(
        context,
        [&]() -> awaitable<void> {
            co_await motors[0].set(7.0f);
            co_await motors[1].set(8.0f);
            co_await motors[0].enable();
            co_await motors[1].enable();
        }(),
        asio::use_future);
    context.run();
    done.get();

    assert(a.last_set == 7.0f);
    assert(b.last_set == 8.0f);
    assert(a.enabled && b.enabled);
    assert(a_ref.angle() == 1.0f);
    assert(b_ref.angle_speed() == 5.0f);
    assert(b_ref.torque() == 6.0f);

    bool unbound_threw = false;
    try {
        static_cast<void>(motor_ref{}.angle());
    } catch (const std::logic_error&) {
        unbound_threw = true;
    }
    assert(unbound_threw);
}

void test_multiton_rejects_duplicate_before_construction() {
    bool duplicate_threw = false;
    try {
        roboctrl::init<fake_motor_a::info_type>({{"same"}, {"same"}});
    } catch (const std::runtime_error&) {
        duplicate_threw = true;
    }
    assert(duplicate_threw);

    const fake_motor_a::info_type info{"registered"};
    roboctrl::init(info);
    motor_ref inferred_ref{info};
    assert(static_cast<bool>(inferred_ref));
}

void test_combined_parser() {
    using namespace roboctrl::utils::byte_literals;
    using parser_type = roboctrl::io::combined_parser<
        roboctrl::io::fixed_data<0xAA_b, 0x55_b>,
        roboctrl::io::nbytes<2>>;

    std::array<std::byte, 4> bytes{0xAA_b, 0x55_b, 0x12_b, 0x34_b};
    parser_type parser;
    assert(parser.parse(bytes) == bytes.size());
    assert(parser.data<1>()[0] == 0x12_b);

    bytes[0] = 0x00_b;
    assert(parser.parse(bytes) == 0);
}

void test_chassis_speed_limit() {
    const auto wheels = roboctrl::ctrl::mecanum_wheel_speeds(
        {.x = 10.0f, .y = -4.0f}, 0.3f, 2.0f, 2.5f);
    const float peak = std::max({
        std::fabs(wheels.left_front),
        std::fabs(wheels.right_front),
        std::fabs(wheels.left_rear),
        std::fabs(wheels.right_rear)});
    assert(peak <= 2.5f + 1e-6f);
}

void test_selected_configuration() {
    roboctrl::config::validate_configuration(
        roboctrl::config::cans,
        roboctrl::config::serials,
        roboctrl::config::dji_motors,
        roboctrl::config::control_pad,
        roboctrl::config::imu,
        roboctrl::config::robot);
}

} // namespace

int main() {
    test_motor_ref_type_erasure();
    test_multiton_rejects_duplicate_before_construction();
    test_combined_parser();
    test_chassis_speed_limit();
    test_selected_configuration();
}
