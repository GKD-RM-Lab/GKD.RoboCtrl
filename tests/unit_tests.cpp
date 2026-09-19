#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <asio.hpp>

#include "config/runtime.hpp"
#include "config/validate.hpp"
#include "core/logger.h"
#include "ctrl/control_mapping.hpp"
#include "device/super_cap.h"
#include "device/motor/base.hpp"
#include "device/imu/base.hpp"
#include "device/controlpad.h"
#include "io/write_queue.hpp"
#include "utils/callback.hpp"
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

[[noreturn]] void fail_check(std::string_view expression,
                             std::string_view file,
                             int line) {
    std::cerr << file << ':' << line << ": check failed: " << expression << '\n';
    std::abort();
}

#define CHECK(expression)                                                        \
    do {                                                                         \
        if (!(expression)) {                                                     \
            fail_check(#expression, __FILE__, __LINE__);                         \
        }                                                                        \
    } while (false)

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

awaitable<void> exercise_motor_polymorphism(motor_base& first, motor_base& second) {
    co_await first.set(7.0f);
    co_await second.set(8.0f);
    co_await first.enable();
    co_await second.enable();
}

void test_motor_base_runtime_polymorphism() {
    fake_motor_a a{{"a"}};
    fake_motor_b b{{"b"}};
    a.set_measurements(1.0f, 2.0f, 3.0f);
    b.set_measurements(4.0f, 5.0f, 6.0f);

    std::array<motor_base*, 2> motors{&a, &b};

    asio::io_context context;
    auto done = asio::co_spawn(
        context,
        exercise_motor_polymorphism(*motors[0], *motors[1]),
        asio::use_future);
    context.run();
    done.get();

    CHECK(a.last_set == 7.0f);
    CHECK(b.last_set == 8.0f);
    CHECK(a.enabled && b.enabled);
    CHECK(motors[0]->angle() == 1.0f);
    CHECK(motors[1]->angle_speed() == 5.0f);
    CHECK(motors[1]->torque() == 6.0f);
}

void test_multiton_rejects_duplicate_before_construction() {
    bool duplicate_threw = false;
    try {
        roboctrl::init<fake_motor_a::info_type>({{"same"}, {"same"}});
    } catch (const std::runtime_error&) {
        duplicate_threw = true;
    }
    CHECK(duplicate_threw);

    const fake_motor_a::info_type info{"registered"};
    roboctrl::init(info);
    motor_base* inferred_motor = &roboctrl::get<fake_motor_a>(info.name);
    CHECK(inferred_motor != nullptr);
}

void test_combined_parser() {
    using namespace roboctrl::utils::byte_literals;
    using parser_type = roboctrl::io::combined_parser<
        roboctrl::io::fixed_data<0xAA_b, 0x55_b>,
        roboctrl::io::nbytes<2>>;

    std::array<std::byte, 4> bytes{0xAA_b, 0x55_b, 0x12_b, 0x34_b};
    parser_type parser;
    CHECK(parser.parse(bytes) == bytes.size());
    CHECK(parser.data<1>()[0] == 0x12_b);

    bytes[0] = 0x00_b;
    CHECK(parser.parse(bytes) == 0);
}

struct fake_keyed_io : roboctrl::io::keyed_io_base<std::uint8_t> {
    using roboctrl::io::keyed_io_base<std::uint8_t>::on_data;

    void emit(std::uint8_t key, roboctrl::io::byte_span data) {
        dispatch(key, data);
    }
};

static_assert(requires(fake_keyed_io& io) {
    io.on_data(1, [](const roboctrl::io::byte_span&) {});
});

struct test_packet {
    std::uint32_t value;
};

static_assert(roboctrl::utils::package<test_packet>);

struct invalid_typed_callback_result {
    int operator()(const test_packet&) const { return 0; }
};

struct invalid_typed_awaitable_result {
    awaitable<int> operator()(const test_packet&) const { co_return 0; }
};

template<typename Fn>
concept keyed_callback_registerable = requires(fake_keyed_io& io, Fn fn) {
    io.on_data(1, fn);
};

static_assert(!keyed_callback_registerable<invalid_typed_callback_result>);
static_assert(!keyed_callback_registerable<invalid_typed_awaitable_result>);

struct fake_bare_io : roboctrl::io::bare_io_base {
    awaitable<void> send(roboctrl::io::byte_span data) {
        co_await roboctrl::yield();
        sent.assign(data.begin(), data.end());
    }

    awaitable<void> task() { co_return; }

    std::vector<std::byte> sent;
};

static_assert(roboctrl::io::bare_io<fake_bare_io>);

std::uint32_t free_function_packet_value = 0;

void receive_test_packet(const test_packet& packet) {
    free_function_packet_value = packet.value;
}

void run_global_context() {
    roboctrl::async::io_context().restart();
    roboctrl::async::run();
}

void test_keyed_io_rejects_conflicting_sizes() {
    fake_keyed_io io;
    io.on_data(1, [](roboctrl::io::byte_span) {}, 2);

    bool threw = false;
    try {
        io.on_data(1, [](roboctrl::io::byte_span) {}, 3);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

void test_callback_and_typed_io_lifetimes() {
    roboctrl::callback<int> callbacks;
    int synchronous_value = 0;
    int asynchronous_value = 0;
    callbacks.add([&](int value) { synchronous_value = value; });
    callbacks.add([&](int value) -> awaitable<void> {
        co_await roboctrl::yield();
        asynchronous_value = value;
    });
    callbacks(42);
    run_global_context();
    CHECK(synchronous_value == 42);
    CHECK(asynchronous_value == 42);

    fake_keyed_io io;
    free_function_packet_value = 0;
    io.on_data(1, &receive_test_packet);

    std::size_t raw_const_ref_size = 0;
    io.on_data(3, [&](const roboctrl::io::byte_span& bytes) -> awaitable<void> {
        co_await roboctrl::yield();
        raw_const_ref_size = bytes.size();
    });

    std::uint32_t coroutine_packet_value = 0;
    io.on_data(2, [&](const test_packet& packet) -> awaitable<void> {
        co_await roboctrl::yield();
        coroutine_packet_value = packet.value;
    });

    const test_packet first{0x12345678U};
    const test_packet second{0xABCDEF01U};
    io.emit(1, roboctrl::utils::to_bytes(first));
    io.emit(2, roboctrl::utils::to_bytes(second));
    io.emit(3, roboctrl::utils::to_bytes(second));
    run_global_context();
    CHECK(free_function_packet_value == first.value);
    CHECK(coroutine_packet_value == second.value);
    CHECK(raw_const_ref_size == sizeof(second));

    fake_bare_io sender;
    const test_packet outgoing{0xC0FFEE11U};
    roboctrl::spawn(roboctrl::io::send(sender, outgoing));
    run_global_context();
    CHECK(sender.sent.size() == sizeof(outgoing));
    CHECK(roboctrl::utils::from_bytes<test_packet>(sender.sent).value == outgoing.value);
}

awaitable<void> enqueue_latest_write_test(
    roboctrl::io::write_queue& queue,
    asio::steady_timer& first_write_gate,
    bool& overflow_rejected) {
    const std::array<std::byte, 1> blocker{std::byte{0x00}};
    const std::array<std::byte, 1> first{std::byte{0x11}};
    const std::array<std::byte, 1> second{std::byte{0x22}};
    const std::array<std::byte, 1> replacement{std::byte{0x33}};

    co_await queue.send_latest(0, blocker);
    co_await roboctrl::yield();
    co_await queue.send_latest(1, first);
    co_await queue.send_latest(2, second);
    co_await queue.send_latest(1, replacement);
    try {
        const std::vector<std::byte> oversized(64 * 1024 + 1, std::byte{0x44});
        co_await queue.send_latest(3, oversized);
    } catch (const roboctrl::io::write_queue_full&) {
        overflow_rejected = true;
    }
    first_write_gate.cancel();
}

awaitable<void> enqueue_one_write(roboctrl::io::write_queue& queue,
                                  std::span<const std::byte> bytes) {
    co_await queue.send(bytes);
}

awaitable<void> expect_latched_write_failure(roboctrl::io::write_queue& queue,
                                             bool& rethrew) {
    try {
        const std::array<std::byte, 1> value{std::byte{0x01}};
        co_await queue.send(value);
    } catch (const std::runtime_error&) {
        rethrew = true;
    }
}

void test_write_queue_replaces_stale_values_and_latches_failures() {
    std::vector<std::vector<std::byte>> writes;
    asio::steady_timer first_write_gate{
        roboctrl::async::executor(), 2s};
    bool first_write = true;
    bool first_write_was_cancelled = false;
    roboctrl::io::write_queue queue{
        [&](std::span<const std::byte> bytes) -> awaitable<void> {
            if (std::exchange(first_write, false)) {
                std::error_code error;
                co_await first_write_gate.async_wait(
                    asio::redirect_error(asio::use_awaitable, error));
                first_write_was_cancelled = error == asio::error::operation_aborted;
            }
            writes.emplace_back(bytes.begin(), bytes.end());
            co_return;
        }};

    bool overflow_rejected = false;
    roboctrl::spawn(enqueue_latest_write_test(queue, first_write_gate, overflow_rejected));
    run_global_context();

    CHECK(overflow_rejected);
    CHECK(first_write_was_cancelled);
    CHECK(writes.size() == 3);
    CHECK(writes[0].front() == std::byte{0x00});
    CHECK(writes[1].front() == std::byte{0x33});
    CHECK(writes[2].front() == std::byte{0x22});

    roboctrl::io::write_queue failing_queue{
        [](std::span<const std::byte>) -> awaitable<void> {
            throw std::runtime_error{"injected writer failure"};
            co_return;
        }};
    const std::array<std::byte, 1> byte{std::byte{0x55}};
    roboctrl::spawn(enqueue_one_write(failing_queue, byte));
    run_global_context();
    CHECK(failing_queue.failed());

    bool rethrew = false;
    roboctrl::spawn(expect_latched_write_failure(failing_queue, rethrew));
    run_global_context();
    CHECK(rethrew);

    std::vector<std::size_t> capacity_write_sizes;
    asio::steady_timer capacity_gate{roboctrl::async::executor(), 2s};
    bool capacity_gate_cancelled = false;
    bool capacity_first_write = true;
    roboctrl::io::write_queue capacity_queue{
        [&](std::span<const std::byte> bytes) -> awaitable<void> {
            if (std::exchange(capacity_first_write, false)) {
                std::error_code error;
                co_await capacity_gate.async_wait(
                    asio::redirect_error(asio::use_awaitable, error));
                capacity_gate_cancelled = error == asio::error::operation_aborted;
            }
            capacity_write_sizes.push_back(bytes.size());
        }};

    bool different_key_rejected = false;
    auto enqueue_capacity_test = [&]() -> awaitable<void> {
        const std::array<std::byte, 1> blocker{std::byte{0x01}};
        const std::vector<std::byte> full(64 * 1024, std::byte{0x22});
        const std::vector<std::byte> safe_replacement(64 * 1024, std::byte{0x00});
        co_await capacity_queue.send_latest(0, blocker);
        co_await roboctrl::yield();
        co_await capacity_queue.send_latest(1, full);
        co_await capacity_queue.send_latest(1, safe_replacement);
        try {
            const std::array<std::byte, 1> other{std::byte{0x33}};
            co_await capacity_queue.send_latest(2, other);
        } catch (const roboctrl::io::write_queue_full&) {
            different_key_rejected = true;
        }
        capacity_gate.cancel();
    };
    roboctrl::spawn(enqueue_capacity_test());
    run_global_context();
    CHECK(capacity_gate_cancelled);
    CHECK(different_key_rejected);
    CHECK(capacity_write_sizes.size() == 2);
    CHECK(capacity_write_sizes[0] == 1);
    CHECK(capacity_write_sizes[1] == 64 * 1024);
}

void test_can_write_queue_keeps_different_dlc() {
    using roboctrl::io::can_write_key;
    std::vector<std::vector<std::byte>> sent;
    asio::steady_timer gate{roboctrl::async::executor(), 2s};
    bool first = true;
    roboctrl::io::write_queue queue{[&](std::span<const std::byte> data) -> awaitable<void> {
        if (std::exchange(first, false)) {
            asio::error_code error;
            co_await gate.async_wait(asio::redirect_error(asio::use_awaitable, error));
        }
        sent.emplace_back(data.begin(), data.end());
    }};
    auto enqueue = [&]() -> awaitable<void> {
        const std::array<std::byte, 1> blocker{std::byte{0}};
        co_await queue.send_latest(can_write_key(0x100, 1), blocker);
        co_await roboctrl::yield();
        auto special = roboctrl::device::motor_protocol::encode_j6006_enabled(true);
        const auto velocity = roboctrl::device::motor_protocol::encode_j6006_velocity(0.f);
        co_await queue.send_latest(can_write_key(0x201, special.size()), special);
        co_await queue.send_latest(can_write_key(0x201, velocity.size()), velocity);
        special = roboctrl::device::motor_protocol::encode_j6006_enabled(false);
        co_await queue.send_latest(can_write_key(0x201, special.size()), special);
        gate.cancel();
    };
    roboctrl::spawn(enqueue());
    run_global_context();
    CHECK(sent.size() == 3);
    CHECK(sent[1].size() == 8 && sent[1].back() == std::byte{0xfd});
    CHECK(sent[2].size() == 4);
}

void test_chassis_speed_limit() {
    const auto wheels = roboctrl::utils::kinematics::inverse_mecanum(
        {.x = 10.0f, .y = -4.0f}, 0.3f, 2.5f);
    const float peak = std::max({
        std::fabs(wheels.left_front),
        std::fabs(wheels.right_front),
        std::fabs(wheels.left_rear),
        std::fabs(wheels.right_rear)});
    CHECK(peak <= 2.5f + 1e-6f);

    const auto base_wheels = roboctrl::utils::kinematics::inverse_mecanum(
        {.x = 1.0f, .y = 0.0f}, 0.0f, 2.5f);
    CHECK(base_wheels.left_front == base_wheels.right_front);
}

void test_device_input_abstractions() {
    roboctrl::device::control_pad_state input;
    input.ch0 = 11;
    input.ch4 = -22;
    input.s1 = 1;
    CHECK(input.channel(roboctrl::device::control_channel::ch0) == 11);
    CHECK(input.gimbal_pitch_wheel() == -22);

    const roboctrl::device::euler_angle angle{.roll = 1.0f, .pitch = 2.0f, .yaw = 3.0f};
    const roboctrl::device::three_axis vector{.x = 4.0f, .y = 5.0f, .z = 6.0f};
    CHECK(angle.yaw == 3.0f && vector.z == 6.0f);
}

void test_pair_aliases_survive_copy_and_move() {
    roboctrl::utils::pair<int> source{1, 2};
    roboctrl::utils::pair<int> copy{source};
    copy.left = 7;
    CHECK(source.p.first == 1);
    CHECK(copy.p.first == 7);
    CHECK(&copy.first == &copy.p.first);
    CHECK(&copy.left == &copy.p.first);

    roboctrl::utils::pair<int> movable{3, 4};
    roboctrl::utils::pair<int> moved{std::move(movable)};
    moved.right = 9;
    CHECK(moved.p.second == 9);
    CHECK(&moved.second == &moved.p.second);
    CHECK(&moved.right == &moved.p.second);
}

void test_logger_routes_errors_to_stderr() {
    std::ostringstream stdout_capture;
    std::ostringstream stderr_capture;
    auto* previous_stdout = std::cout.rdbuf(stdout_capture.rdbuf());
    auto* previous_stderr = std::cerr.rdbuf(stderr_capture.rdbuf());

    roboctrl::logger::set_level(roboctrl::log_level::Debug);
    roboctrl::logger::instance().log_info("unit-test-info-sentinel");
    roboctrl::logger::instance().log_error("unit-test-error-sentinel");

    std::cout.rdbuf(previous_stdout);
    std::cerr.rdbuf(previous_stderr);
    roboctrl::logger::set_level(roboctrl::log_level::Info);

    CHECK(stdout_capture.str().contains("unit-test-info-sentinel"));
    CHECK(!stdout_capture.str().contains("unit-test-error-sentinel"));
    CHECK(stderr_capture.str().contains("unit-test-error-sentinel"));
}

void test_super_cap_is_constructible_and_initially_offline() {
    static_assert(std::is_default_constructible_v<roboctrl::device::super_cap>);
    auto& cap = roboctrl::device::super_cap::instance();
    CHECK(cap.offline());
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
    pid.update(1.0f, 0.0f, 0.1f);
    CHECK(pid.target() == 1.0f);
    CHECK(std::fabs(pid.state() - 1.1f) < 1e-6f);

    roboctrl::utils::ramp_f ramp{{.acc = 2.0f}};
    ramp.update(10.0f, 0.5f);
    CHECK(std::fabs(ramp.state() - 1.0f) < 1e-6f);

    roboctrl::utils::runtime_control_chain<roboctrl::fp32> chain;
    chain.add(std::make_unique<add_stage>(1.0f));
    chain.add(std::make_unique<add_stage>(2.0f));
    CHECK(std::fabs(chain.update(3.0f, 0.01f) - 6.0f) < 1e-6f);

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
    CHECK(value == 4.0f);
}

void test_matrix_and_rls_initialization() {
    roboctrl::utils::Matrixf<2, 2> lhs;
    lhs[0][0] = 1.0f;
    lhs[1][1] = 1.0f;
    roboctrl::utils::Matrixf<2, 2> rhs;
    rhs[0][1] = 2.0f;
    rhs[1][0] = 3.0f;
    const auto product = lhs * rhs;
    CHECK(product[0][1] == 2.0f);
    CHECK(product[1][0] == 3.0f);

    const auto row = product.row(0);
    const auto col = product.col(1);
    CHECK(row[0][1] == 2.0f);
    CHECK(col[0][0] == 2.0f && col[1][0] == 0.0f);

    roboctrl::utils::Matrix<2, 2, double> needs_pivot;
    needs_pivot[0][1] = 1.0;
    needs_pivot[1][0] = 1.0;
    const auto inverse = needs_pivot.inv();
    CHECK(inverse[0][0] == 0.0);
    CHECK(inverse[0][1] == 1.0);
    CHECK(inverse[1][0] == 1.0);
    CHECK(inverse[1][1] == 0.0);

    roboctrl::utils::Matrix<2, 3, double> rectangular;
    static_assert(std::same_as<
                  decltype(rectangular.trans()),
                  roboctrl::utils::Matrix<3, 2, double>>);

    roboctrl::utils::Matrix<2, 1, double> diagonal_values;
    diagonal_values[0][0] = 1.25;
    diagonal_values[1][0] = 2.5;
    const auto diagonal = roboctrl::utils::Matrix<2, 2, double>::diag(diagonal_values);
    static_assert(std::same_as<
                  std::remove_cv_t<decltype(diagonal)>,
                  roboctrl::utils::Matrix<2, 2, double>>);
    CHECK(diagonal[0][0] == 1.25);
    CHECK(diagonal[1][1] == 2.5);

    roboctrl::utils::RLS<2> rls{1.0f, 0.99f};
    CHECK(rls.getOutput() == 0.0f);
    auto& params = rls.getParamsVector();
    CHECK(params[0][0] == 0.0f && params[1][0] == 0.0f);

    bool invalid_lambda = false;
    try {
        roboctrl::utils::RLS<2> invalid{1.0f, 0.0f};
    } catch (const std::invalid_argument&) {
        invalid_lambda = true;
    }
    CHECK(invalid_lambda);

    roboctrl::utils::RLS<1> scalar_rls{1.0f, 1.0f};
    roboctrl::utils::Matrixf<1, 1> sample;
    sample[0][0] = 1.0f;
    scalar_rls.update(sample, 2.0f);
    CHECK(std::fabs(scalar_rls.getOutput() - 1.0f) < 1e-6f);
    CHECK(std::fabs(scalar_rls.getParamsVector()[0][0] - 1.0f) < 1e-6f);
    scalar_rls.reset();
    CHECK(scalar_rls.getOutput() == 0.0f);
    CHECK(scalar_rls.getParamsVector()[0][0] == 0.0f);
}

void test_control_mapping_requires_arm_and_preserves_edges() {
    roboctrl::ctrl::control_mapper mapper;
    roboctrl::device::control_pad_state input;
    input.ch3 = 660;
    auto command = mapper.update(input);
    CHECK(command.velocity.x == 0.0f);
    CHECK(!mapper.armed());

    input = {.ch4 = -660, .s1 = 2, .s2 = 2};
    command = mapper.update(input);
    CHECK(command.arm_requested);
    CHECK(mapper.armed());

    input = {.ch1 = 330, .ch2 = -330, .ch3 = 660, .ch4 = 660, .s1 = 1, .s2 = 1};
    command = mapper.update(input);
    CHECK(std::fabs(command.velocity.x - 3.0f) < 1e-6f);
    CHECK(std::fabs(command.velocity.y + 1.5f) < 1e-6f);
    CHECK(command.rotate_speed == 1.0f);
    CHECK(command.friction_enabled);
    CHECK(command.firing);
    CHECK(command.use_pitch_target);

    input = {.key = 0x40};
    command = mapper.update(input);
    CHECK(command.rotate_speed == 1.0f);
    command = mapper.update(input);
    CHECK(command.rotate_speed == 1.0f);
    input.key = 0;
    mapper.update(input);
    input.key = 0x40;
    command = mapper.update(input);
    CHECK(command.rotate_speed == 0.0f);

    mapper.reset();
    command = mapper.update({.ch3 = 660});
    CHECK(!mapper.armed());
    CHECK(command.velocity.x == 0.0f);
}

void test_control_mapping_requires_a_fresh_arm_edge() {
    roboctrl::ctrl::control_mapper mapper;
    const roboctrl::device::control_pad_state neutral{};
    const roboctrl::device::control_pad_state unlock{
        .ch4 = -660,
        .s1 = 2,
        .s2 = 2,
    };

    mapper.update(neutral);
    auto command = mapper.update(unlock);
    CHECK(command.arm_requested);
    command = mapper.update(unlock);
    CHECK(!command.arm_requested);

    mapper.reset();
    command = mapper.update(unlock);
    CHECK(!command.arm_requested);
    CHECK(!mapper.armed());

    mapper.update(neutral);
    command = mapper.update(unlock);
    CHECK(command.arm_requested);
    CHECK(mapper.armed());
}

void test_rejects_invalid_control_configuration() {
    auto config_directory = std::filesystem::current_path();
    while (!std::filesystem::exists(config_directory / "configs")) {
        const auto parent = config_directory.parent_path();
        CHECK(parent != config_directory);
        config_directory = parent;
    }
    const auto loaded = roboctrl::config::load_configuration(config_directory / "configs/infantry.yaml");
    CHECK(loaded);
    roboctrl::config::validate_configuration(
        loaded->cans,
        loaded->serials,
        loaded->dji_motors,
        loaded->control_pad,
        loaded->imu,
        loaded->robot);
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
    CHECK(threw);

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
    CHECK(threw);

    const auto rejected = [&](const auto& cans,
                              const auto& serials,
                              const auto& motors,
                              const auto& robot) {
        try {
            roboctrl::config::validate_configuration(
                std::span{cans},
                std::span{serials},
                std::span{motors},
                loaded->control_pad,
                loaded->imu,
                robot);
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    };

    auto duplicate_can = loaded->cans;
    duplicate_can[1].interface_name = duplicate_can[0].interface_name;
    CHECK(rejected(duplicate_can, loaded->serials, loaded->dji_motors, loaded->robot));

    auto duplicate_serial = loaded->serials;
    auto serial_alias = duplicate_serial.front();
    serial_alias.name = "serial-alias";
    duplicate_serial.push_back(std::move(serial_alias));
    CHECK(rejected(loaded->cans, duplicate_serial, loaded->dji_motors, loaded->robot));

    auto unknown_motor_type = loaded->dji_motors;
    unknown_motor_type.front().type_ =
        static_cast<roboctrl::device::dji_motor::type>(2007);
    CHECK(rejected(loaded->cans, loaded->serials, unknown_motor_type, loaded->robot));

    auto out_of_range_current = loaded->dji_motors;
    const auto m3508 = std::find_if(
        out_of_range_current.begin(), out_of_range_current.end(), [](const auto& motor) {
            return motor.type_ == roboctrl::device::dji_motor::M3508;
        });
    CHECK(m3508 != out_of_range_current.end());
    m3508->pid_params.max_out = 20000.0f;
    CHECK(rejected(
        loaded->cans, loaded->serials, out_of_range_current, loaded->robot));

    auto invalid_gm6020_id = loaded->dji_motors;
    const auto gm6020 = std::find_if(
        invalid_gm6020_id.begin(), invalid_gm6020_id.end(), [](const auto& motor) {
            return motor.type_ == roboctrl::device::dji_motor::M6020;
        });
    CHECK(gm6020 != invalid_gm6020_id.end());
    gm6020->id = 8;
    CHECK(rejected(
        loaded->cans, loaded->serials, invalid_gm6020_id, loaded->robot));

    auto duplicate_chassis_role = loaded->robot;
    duplicate_chassis_role.chassis_info.right_front_motor =
        duplicate_chassis_role.chassis_info.left_front_motor;
    CHECK(rejected(
        loaded->cans, loaded->serials, loaded->dji_motors, duplicate_chassis_role));

    auto duplicate_cross_subsystem_role = loaded->robot;
    duplicate_cross_subsystem_role.shoot_info.trigger_motor =
        duplicate_cross_subsystem_role.gimbal_info.yaw_motor_key;
    CHECK(rejected(
        loaded->cans, loaded->serials, loaded->dji_motors,
        duplicate_cross_subsystem_role));
}

void test_runtime_configuration_files() {
    auto config_directory = std::filesystem::current_path();
    while (!std::filesystem::exists(config_directory / "configs")) {
        const auto parent = config_directory.parent_path();
        CHECK(parent != config_directory);
        config_directory = parent;
    }
    for (const auto* path : {
             "configs/infantry.yaml",
             "configs/hero.yaml",
             "configs/sentry.yaml",
             "configs/project.yaml",
        }) {
        auto config = roboctrl::config::load_configuration(config_directory / path);
        if (std::string_view{path} == "configs/sentry.yaml") {
            // The source topology contains a confirmed 0x201 conflict; no guessed CAN remap.
            CHECK(!config);
            CHECK(config.error().find("overlaps a motor feedback ID") != std::string::npos);
            const auto text = roboctrl::config::read_text_file(config_directory / path);
            CHECK(text);
            auto parsed = rfl::yaml::read<roboctrl::config::runtime_config, rfl::NoExtraFields,
                rfl::DefaultIfMissing>(*text);
            CHECK(parsed);
            config = std::move(*parsed); // Structural assertions only, not deployment validity.
        }
        if (!config) {
            std::cerr << path << ": " << config.error() << '\n';
        }
        CHECK(config);
        CHECK(config->schema_version == 1);
        CHECK(!config->cans.empty());
        CHECK(!config->dji_motors.empty());
        if (std::string_view{path} != "configs/sentry.yaml") roboctrl::config::validate_configuration(
            config->cans,
            config->serials,
            config->dji_motors,
            config->control_pad,
            config->imu,
            config->robot);
        if (std::string_view{path} == "configs/sentry.yaml") {
            CHECK(config->cans.size() == 3);
            CHECK(config->serials.size() == 3);
            CHECK(config->dji_motors.size() == 9);
            CHECK(config->j6006_motors.size() == 1);
            CHECK(config->additional_imus.size() == 1);
            CHECK(config->robot.gimbal_info.imu_key == "imu_head");
            CHECK(config->robot.large_yaw_info);
            CHECK(!config->robot.large_yaw_info->yaw_zero_calibrated);
            CHECK(config->robot.large_yaw_info->yaw_motor_type == "j6006");
            CHECK(config->robot.enable_chassis);
            CHECK(config->robot.enable_gimbal);
            CHECK(config->robot.enable_shoot);
        }
    }
}

void test_runtime_default_configuration_path() {
    CHECK(roboctrl::config::default_configuration_path() ==
           std::filesystem::path{"configs/infantry.yaml"});
}

void test_async_exceptions_use_the_shutdown_handler() {
    roboctrl::task_context context;
    CHECK(!context.failed());
    bool shutdown_ran = false;
    context.set_shutdown_handler([&]() -> awaitable<void> {
        shutdown_ran = true;
        co_await roboctrl::yield();
    });
    context.post([] { throw std::runtime_error{"injected posted-task failure"}; });
    context.run();
    CHECK(shutdown_ran);
    CHECK(context.failed());
    CHECK(context.shutdown_requested());
}

} // namespace

void test_motor_protocols();
void test_network_protocols();
void run_referee_protocol_tests();
void test_power_control();
void run_ballistics_tests();
void run_shoot_integration_tests();
void run_motion_migration_tests();
void run_configuration_migration_tests();

int main() {
    test_motor_protocols();
    test_network_protocols();
    run_referee_protocol_tests();
    test_power_control();
    run_ballistics_tests();
    run_shoot_integration_tests();
    run_motion_migration_tests();
    run_configuration_migration_tests();

    test_motor_base_runtime_polymorphism();
    test_multiton_rejects_duplicate_before_construction();
    test_combined_parser();
    test_keyed_io_rejects_conflicting_sizes();
    test_callback_and_typed_io_lifetimes();
    test_write_queue_replaces_stale_values_and_latches_failures();
    test_chassis_speed_limit();
    test_can_write_queue_keeps_different_dlc();
    test_device_input_abstractions();
    test_pair_aliases_survive_copy_and_move();
    test_logger_routes_errors_to_stderr();
    test_super_cap_is_constructible_and_initially_offline();
    test_control_chain_and_explicit_dt();
    test_matrix_and_rls_initialization();
    test_control_mapping_requires_arm_and_preserves_edges();
    test_control_mapping_requires_a_fresh_arm_edge();
    test_rejects_invalid_control_configuration();
    test_runtime_configuration_files();
    test_runtime_default_configuration_path();
    test_async_exceptions_use_the_shutdown_handler();
}
