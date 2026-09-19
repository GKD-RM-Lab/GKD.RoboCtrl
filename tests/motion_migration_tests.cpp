#include <array>
#include <bit>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "ctrl/motion_logic.hpp"
#include "ctrl/control_mapping.hpp"
#include "device/gimbal/gkd_sentry_gimbal.hpp"
#include "device/imu/serial_imu_codec.hpp"

using namespace roboctrl;

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
bool near(float a, float b) { return std::abs(a - b) < 1e-4f; }
struct fake_imu : device::imu_base {
    fake_imu() : imu_base{1s} {}
    void feedback(device::euler_angle angle, device::three_axis rate) { angle_ = angle; gyro_ = rate; tick(); }
    void expire() { tick_time_ = utils::now() - 2s; }
};
struct fake_motor : device::motor_base {
    fake_motor() : motor_base{1s, 1} {}
    bool enabled{}, current_supported{true}, reported_fault{};
    float current{}, speed{};
    awaitable<void> set(float target) override { speed = target; co_return; }
    awaitable<void> set_angle_speed(float target) override { speed = target; co_return; }
    awaitable<void> set_current(float target) override { current = target; co_return; }
    bool supports_current_control() const override { return current_supported; }
    bool faulted() const override { return reported_fault; }
    awaitable<void> enable() override { enabled = true; co_return; }
    void set_enabled(bool value) override { enabled = value; if (!value) current = speed = 0; }
    void feedback(float angle, float speed_value = 99) { angle_ = angle; angle_speed_ = speed_value; tick(); }
};
void run(awaitable<void> task) {
    asio::io_context context;
    std::exception_ptr error;
    asio::co_spawn(context, std::move(task), [&](std::exception_ptr value) { error = value; });
    context.run();
    if (error) std::rethrow_exception(error);
}

void test_coordinates_and_follow() {
    const auto out = ctrl::gimbal_to_chassis({.x=1, .y=0}, Pi_f / 2);
    require(near(out.x, 0) && near(out.y, -1), "legacy quarter-turn transform");
    const auto back = ctrl::gimbal_to_chassis(out, -Pi_f / 2);
    require(near(back.x, 1) && near(back.y, 0), "transform inverse");
    ctrl::chassis_follow follow;
    follow.configure({.kp=2, .max_out=4}, 1, .7f, .005f);
    require(near(follow.update(0, .2f, .002f), -.4f), "follow PID active");
    require(near(follow.update(1, 1, .002f), 1), "spin request preserved");
    require(near(follow.update(0, .4f, .002f), .7f), "stopped spin keeps direction");
    require(near(follow.update(0, -.02f, .002f), .04f), "crossed zero returns to PID");
    follow.update(-1, -3.12f, .002f);
    require(near(follow.update(0, 3.12f, .002f), -.7f), "pi wrap is not a zero crossing");
    follow.reset();
    require(near(follow.update(0, .1f, .002f), -.2f), "reset removes spin state");
}

void test_imu_codec() {
    std::array<std::byte,24> bytes{};
    const std::array<float,6> values{180,-30,15,1000,2000,-3000};
    for (std::size_t i=0; i<values.size(); ++i) {
        const auto bits = std::bit_cast<std::uint32_t>(values[i]);
        for (std::size_t j=0; j<4; ++j) bytes[4*i+j] = std::byte((bits >> (8*j)) & 0xff);
    }
    const auto parsed = device::decode_serial_imu(bytes);
    require(parsed && parsed->pitch == -30 && parsed->roll_v == -3000, "IMU little-endian vector");
    require(!device::decode_serial_imu(std::span(bytes).first(23)), "IMU short payload rejected");
    bytes[0]=std::byte{0}; bytes[1]=std::byte{0}; bytes[2]=std::byte{0x80}; bytes[3]=std::byte{0x7f};
    require(!device::decode_serial_imu(bytes), "IMU infinity rejected");
    const float largest = std::numeric_limits<float>::max();
    const auto huge_angles = device::convert_serial_imu({largest,largest,largest,0,0,0},
        {1,-1,1}, {1,1,1}, Pi_f / 180 / 1000);
    require(huge_angles && std::isfinite(huge_angles->angle.yaw) && std::isfinite(huge_angles->angle.pitch),
        "large finite raw angles cannot overflow radians conversion");
    require(!device::convert_serial_imu({0,0,0,largest,0,0}, {1,1,1}, {1,1,1}, 2),
        "finite raw gyro that overflows configured conversion is rejected");
    require(near(device::compensated_yaw_rate(Pi_f/2, {.x=2,.y=8,.z=5}), -2), "yaw gyro pitch projection");
}

void test_gimbal_output_and_gates() {
    fake_imu imu; fake_motor yaw, pitch;
    imu.feedback({}, {.x=0,.y=.1f,.z=1}); yaw.feedback(1); pitch.feedback(0);
    device::gimbal_base::info_type info;
    info.yaw_zero=1; info.yaw_zero_calibrated=true; info.recenter_on_enable=false;
    info.yaw_angle_pid={.kp=4,.max_out=10}; info.pitch_angle_pid={.kp=3,.max_out=10};
    info.yaw_rate_pid={.kp=2,.max_out=100}; info.pitch_rate_pid={.kp=2,.max_out=100};
    device::gkd_sentry_gimbal gimbal;
    require(gimbal.init(info, imu, yaw, &pitch), "fake dependencies bind");
    run(gimbal.update(.001f));
    require(yaw.current==0 && pitch.current==0 && !gimbal.initialized(), "NoForce has zero current");
    gimbal.set_enabled(true); run(gimbal.update(.001f));
    gimbal.set_target_yaw(.5f); gimbal.set_target_pitch(.2f); run(gimbal.update(.001f));
    require(near(yaw.current,2) && near(pitch.current,1), "IMU-rate current cascade bypasses motor speed feedback");
    require(near(gimbal.relative_yaw(),0) && gimbal.relative_yaw_valid(), "mechanical zero used");
    require(near(gimbal.yaw(),0) && near(gimbal.target_yaw(),.5f), "measured and target yaw distinguished");
    imu.feedback({.yaw=std::numeric_limits<float>::quiet_NaN()}, {});
    run(gimbal.update(.001f));
    require(yaw.current==0 && pitch.current==0 && !gimbal.online() && !gimbal.initialized(),
        "nonfinite live feedback cannot permit output or fire readiness");
    require(std::isfinite(gimbal.yaw()) && std::isfinite(gimbal.target_yaw()), "invalid sample does not poison held pose");
    imu.feedback({}, {}); run(gimbal.update(.001f));
    imu.expire(); run(gimbal.update(.001f));
    require(yaw.current==0 && pitch.current==0 && !gimbal.initialized(), "IMU loss zeroes outputs");
    gimbal.set_enabled(false);
    require(!yaw.enabled && !pitch.enabled, "disable reaches bound motors");
}

void test_recenter_and_native_yaw() {
    fake_imu imu; fake_motor yaw, pitch;
    imu.feedback({}, {}); yaw.feedback(1); pitch.feedback(0);
    device::gimbal_base::info_type info;
    info.yaw_zero=1; info.yaw_zero_calibrated=true; info.init_settle_time=2ms;
    info.yaw_relative_pid={.kp=2,.max_out=10};
    info.yaw_rate_pid={.kp=3,.max_out=100};
    device::gkd_sentry_gimbal gimbal;
    require(gimbal.init(info,imu,yaw,&pitch), "recenter binds");
    gimbal.set_recentering(true);
    run(gimbal.update(.01f)); require(!gimbal.initialized(), "disabled dwell does not initialize");
    gimbal.set_enabled(true);
    run(gimbal.update(.001f)); require(!gimbal.initialized(), "dwell requires duration");
    yaw.feedback(1.2f); run(gimbal.update(.001f));
    require(near(yaw.current,-1.2f) && !gimbal.initialized(), "relative PID commands mechanical recenter");
    yaw.feedback(1); run(gimbal.update(.001f)); require(!gimbal.initialized(), "out-of-tolerance resets dwell");
    run(gimbal.update(.001f)); require(gimbal.initialized(), "continuous dwell completes");
    gimbal.set_enabled(false); require(!gimbal.initialized(), "disable resets initialization");

    fake_motor big; big.current_supported=false; big.feedback(1);
    device::gkd_sentry_gimbal native;
    info.yaw_only=true; info.yaw_current_control=false; info.yaw_zero_calibrated=false;
    info.yaw_angle_pid={.kp=4,.max_out=10};
    require(native.init(info,imu,big,nullptr), "native yaw-only binds without phantom pitch");
    native.set_enabled(true); native.set_recentering(true);
    require(!big.enabled, "native drive not enabled before dependency checks");
    run(native.update(.1f));
    require(big.speed==0 && !big.enabled && !native.initialized(), "uncalibrated recenter disables native drive");
    native.set_recentering(false); run(native.update(.001f));
    native.set_target_yaw(.5f); // Native drive handles its internal speed loop.
    run(native.update(.001f)); require(near(big.speed,2) && big.enabled && native.initialized(), "native speed target reaches enabled actuator");
    imu.feedback({}, {.x=std::numeric_limits<float>::infinity()}); run(native.update(.001f));
    require(!big.enabled && !native.online(), "invalid live gyro disables native hold torque");
    imu.feedback({}, {}); run(native.update(.001f));
    imu.expire(); run(native.update(.001f));
    require(!big.enabled && big.speed==0 && !native.online(), "IMU loss disables native hold torque");
    imu.feedback({}, {}); run(native.update(.001f));
    require(big.enabled && native.initialized(), "valid dependency recovery re-enables native drive");
    big.reported_fault=true; run(native.update(.001f));
    require(!big.enabled && !native.online() && !native.initialized(), "drive fault with live heartbeat blocks readiness");
    big.reported_fault=false; run(native.update(.001f));
    require(!big.enabled && !native.online(), "drive fault stays latched until explicit controller disable");
    native.set_enabled(false); native.set_enabled(true); run(native.update(.001f));
    require(big.enabled && native.online(), "explicit rearm clears local fault latch after fault recovery");
    device::gkd_sentry_gimbal invalid;
    info.yaw_current_control=true;
    require(!invalid.init(info,imu,big,nullptr), "unsupported current mode rejected");
}

void test_pid_time_conversion() {
    utils::linear_pid legacy{{.kp=2,.ki=.1f,.kd=3,.max_out=100,.max_iout=10}};
    utils::linear_pid seconds{{.kp=2,.ki=100,.kd=.003f,.max_out=100,.max_iout=10}};
    for (float value : {0.0f,.1f,.2f,.4f}) {
        legacy.update(1,value,1); seconds.update(1,value,.001f);
        require(near(legacy.state(),seconds.state()), "per-step PID conversion ki/dt kd*dt");
    }
}
}
void run_motion_migration_tests() {
    test_coordinates_and_follow(); test_imu_codec(); test_gimbal_output_and_gates();
    test_recenter_and_native_yaw(); test_pid_time_conversion();
    std::cout << "motion migration tests passed\n";
}

#ifdef ROBOCTRL_MOTION_TEST_MAIN
int main() { run_motion_migration_tests(); }
#endif
