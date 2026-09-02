#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <asio.hpp>

#include "config/runtime.hpp"
#include "config/validate.hpp"
#include "ctrl/control_mapping.hpp"
#include "ctrl/shoot_logic.hpp"
#include "device/motor/base.hpp"
#include "device/imu/base.hpp"
#include "device/controlpad.h"
#include "utils/kinematics/mecanum.hpp"
#include "utils/pid.h"
#include "utils/ramp.hpp"
#include "utils/RLS.hpp"
#include "utils/controller.hpp"
#include "io/base.hpp"

using namespace std::chrono_literals;
using roboctrl::awaitable;
using roboctrl::device::motor_base;

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
void test_motor_base_runtime_polymorphism() {
    fake_motor_a a{{"a"}};
    fake_motor_b b{{"b"}};
    a.set_measurements(1.0f, 2.0f, 3.0f);
    b.set_measurements(4.0f, 5.0f, 6.0f);

    std::array<motor_base*, 2> motors{&a, &b};

    asio::io_context context;
    auto done = asio::co_spawn(
        context,
        [&]() -> awaitable<void> {
            co_await motors[0]->set(7.0f);
            co_await motors[1]->set(8.0f);
            co_await motors[0]->enable();
            co_await motors[1]->enable();
        }(),
        asio::use_future);
    context.run();
    done.get();

    assert(a.last_set == 7.0f);
    assert(b.last_set == 8.0f);
    assert(a.enabled && b.enabled);
    assert(motors[0]->angle() == 1.0f);
    assert(motors[1]->angle_speed() == 5.0f);
    assert(motors[1]->torque() == 6.0f);
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
    motor_base* inferred_motor = &roboctrl::get<fake_motor_a>(info.name);
    assert(inferred_motor != nullptr);
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

struct fake_keyed_io : roboctrl::io::keyed_io_base<std::uint8_t> {
    using roboctrl::io::keyed_io_base<std::uint8_t>::on_data;
};

void test_keyed_io_rejects_conflicting_sizes() {
    fake_keyed_io io;
    io.on_data(1, [](roboctrl::io::byte_span) {}, 2);

    bool threw = false;
    try {
        io.on_data(1, [](roboctrl::io::byte_span) {}, 3);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

void test_chassis_speed_limit() {
    const auto wheels = roboctrl::utils::kinematics::inverse_mecanum(
        {.x = 10.0f, .y = -4.0f}, 0.3f, 2.5f);
    const float peak = std::max({
        std::fabs(wheels.left_front),
        std::fabs(wheels.right_front),
        std::fabs(wheels.left_rear),
        std::fabs(wheels.right_rear)});
    assert(peak <= 2.5f + 1e-6f);

    const auto base_wheels = roboctrl::utils::kinematics::inverse_mecanum(
        {.x = 1.0f, .y = 0.0f}, 0.0f, 2.5f);
    assert(base_wheels.left_front == base_wheels.right_front);
}

void test_device_input_abstractions() {
    roboctrl::device::control_pad_state input;
    input.ch0 = 11;
    input.ch4 = -22;
    input.s1 = 1;
    assert(input.channel(roboctrl::device::control_channel::ch0) == 11);
    assert(input.gimbal_pitch_wheel() == -22);

    const roboctrl::device::euler_angle angle{.roll = 1.0f, .pitch = 2.0f, .yaw = 3.0f};
    const roboctrl::device::three_axis vector{.x = 4.0f, .y = 5.0f, .z = 6.0f};
    assert(angle.yaw == 3.0f && vector.z == 6.0f);
}

struct add_stage final : roboctrl::utils::control_stage<roboctrl::fp32> {
    explicit add_stage(roboctrl::fp32 amount) : amount{amount} {}
    roboctrl::fp32 update(roboctrl::fp32 input, roboctrl::fp32) override { return input + amount; }
    void reset() override {}
    roboctrl::fp32 amount;
};

void test_control_chain_and_explicit_dt() {
    roboctrl::utils::linear_pid pid{{.kp = 1.0f, .ki = 1.0f, .kd = 0.0f,
                                     .max_out = 10.0f, .max_iout = 10.0f}};
    pid.set_target(1.0f);
    pid.update(0.0f, 0.1f);
    assert(std::fabs(pid.state() - 1.1f) < 1e-6f);

    roboctrl::utils::ramp_f ramp{{.acc = 2.0f}};
    ramp.update(10.0f, 0.5f);
    assert(std::fabs(ramp.state() - 1.0f) < 1e-6f);

    roboctrl::utils::runtime_control_chain<roboctrl::fp32> chain;
    chain.add(std::make_unique<add_stage>(1.0f));
    chain.add(std::make_unique<add_stage>(2.0f));
    assert(std::fabs(chain.update(3.0f, 0.01f) - 6.0f) < 1e-6f);

    struct value_controller {
        using state_type = float;
        using input_type = float;
        struct params_type {};
        explicit value_controller(params_type) {}
        void update(float input) { state_ = input; }
        float state() const { return state_; }
        float state_ {0.0f};
    };
    roboctrl::utils::control_chain<value_controller> value_chain{
        value_controller{{}}};
    value_chain.update(4.0f);
    const auto value = value_chain.state();
    assert(value == 4.0f);
}

void test_matrix_and_rls_initialization() {
    roboctrl::utils::Matrixf<2, 2> lhs;
    lhs[0][0] = 1.0f;
    lhs[1][1] = 1.0f;
    roboctrl::utils::Matrixf<2, 2> rhs;
    rhs[0][1] = 2.0f;
    rhs[1][0] = 3.0f;
    const auto product = lhs * rhs;
    assert(product[0][1] == 2.0f);
    assert(product[1][0] == 3.0f);

    const auto row = product.row(0);
    const auto col = product.col(1);
    assert(row[0][1] == 2.0f);
    assert(col[0][0] == 2.0f && col[1][0] == 0.0f);

    roboctrl::utils::RLS<2> rls{1.0f, 0.99f};
    assert(rls.getOutput() == 0.0f);
    auto& params = rls.getParamsVector();
    assert(params[0][0] == 0.0f && params[1][0] == 0.0f);

    bool invalid_lambda = false;
    try {
        roboctrl::utils::RLS<2> invalid{1.0f, 0.0f};
    } catch (const std::invalid_argument&) {
        invalid_lambda = true;
    }
    assert(invalid_lambda);
}

void test_control_mapping_requires_arm_and_preserves_edges() {
    roboctrl::ctrl::control_mapper mapper;
    roboctrl::device::control_pad_state input;
    input.ch3 = 660;
    auto command = mapper.update(input);
    assert(command.velocity.x == 0.0f);
    assert(!mapper.armed());

    input = {.ch4 = -660, .s1 = 2, .s2 = 2};
    command = mapper.update(input);
    assert(command.arm_requested);
    assert(mapper.armed());

    input = {.ch1 = 330, .ch2 = -330, .ch3 = 660, .ch4 = 660, .s1 = 1, .s2 = 1};
    command = mapper.update(input);
    assert(std::fabs(command.velocity.x - 3.0f) < 1e-6f);
    assert(std::fabs(command.velocity.y + 1.5f) < 1e-6f);
    assert(command.rotate_speed == 1.0f);
    assert(command.friction_enabled);
    assert(command.firing);
    assert(command.use_pitch_target);

    input = {.key = 0x40};
    command = mapper.update(input);
    assert(command.rotate_speed == 1.0f);
    command = mapper.update(input);
    assert(command.rotate_speed == 1.0f);
    input.key = 0;
    mapper.update(input);
    input.key = 0x40;
    command = mapper.update(input);
    assert(command.rotate_speed == 0.0f);

    mapper.reset();
    command = mapper.update({.ch3 = 660});
    assert(!mapper.armed());
    assert(command.velocity.x == 0.0f);
}

void test_shoot_interlocks() {
    using roboctrl::ctrl::trigger_feed_allowed;
    using roboctrl::ctrl::trigger_jammed;
    assert(trigger_jammed(4500.0f, 0.5f, 4000.0f, 1.0f));
    assert(!trigger_jammed(3500.0f, 0.5f, 4000.0f, 1.0f));
    assert(trigger_feed_allowed(true, true, true, true, false));
    assert(!trigger_feed_allowed(true, true, true, true, true));
    assert(!trigger_feed_allowed(true, false, true, true, false));
}

void test_rejects_invalid_control_configuration() {
    auto config_directory = std::filesystem::current_path();
    while (!std::filesystem::exists(config_directory / "configs")) {
        const auto parent = config_directory.parent_path();
        assert(parent != config_directory);
        config_directory = parent;
    }
    const auto loaded = roboctrl::config::load_configuration(config_directory / "configs/infantry.yaml");
    assert(loaded);
    auto invalid_robot = loaded->robot;
    invalid_robot.chassis_info.control_time = 0ns;

    bool threw = false;
    try {
        roboctrl::config::validate_configuration(
            loaded->cans,
            loaded->serials,
            loaded->dji_motors,
            loaded->control_pad,
            loaded->imu,
            invalid_robot);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);

    auto invalid_pid = loaded->dji_motors;
    invalid_pid.front().pid_params.max_out = -1.0f;
    threw = false;
    try {
        roboctrl::config::validate_configuration(
            loaded->cans,
            loaded->serials,
            invalid_pid,
            loaded->control_pad,
            loaded->imu,
            loaded->robot);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

void test_runtime_configuration_files() {
    auto config_directory = std::filesystem::current_path();
    while (!std::filesystem::exists(config_directory / "configs")) {
        const auto parent = config_directory.parent_path();
        assert(parent != config_directory);
        config_directory = parent;
    }
    for (const auto* path : {
             "configs/infantry.yaml",
             "configs/hero.yaml",
             "configs/sentry.yaml",
             "configs/project.yaml",
        }) {
        const auto config = roboctrl::config::load_configuration(config_directory / path);
        if (!config) {
            std::cerr << path << ": " << config.error() << '\n';
        }
        assert(config);
        assert(config->schema_version == 1);
        assert(!config->cans.empty());
        assert(!config->dji_motors.empty());
        if (std::string_view{path} == "configs/sentry.yaml") {
            assert(config->cans.size() == 3);
            assert(config->serials.size() == 2);
            assert(config->dji_motors.size() == 11);
            assert(config->robot.enable_chassis);
            assert(config->robot.enable_gimbal);
            assert(config->robot.enable_shoot);
        }
    }
}

void test_runtime_default_configuration_path() {
    assert(roboctrl::config::default_configuration_path() ==
           std::filesystem::path{"configs/infantry.yaml"});
}

} // namespace

int main() {
    test_motor_base_runtime_polymorphism();
    test_multiton_rejects_duplicate_before_construction();
    test_combined_parser();
    test_keyed_io_rejects_conflicting_sizes();
    test_chassis_speed_limit();
    test_device_input_abstractions();
    test_control_chain_and_explicit_dt();
    test_matrix_and_rls_initialization();
    test_control_mapping_requires_arm_and_preserves_edges();
    test_shoot_interlocks();
    test_rejects_invalid_control_configuration();
    test_runtime_configuration_files();
    test_runtime_default_configuration_path();
}
