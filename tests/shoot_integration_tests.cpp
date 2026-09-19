#include "ctrl/shoot.h"
#include "device/referee/referee.h"
#include <future>
#include <limits>
#include <stdexcept>
#include <string_view>

using namespace std::chrono_literals;
using namespace roboctrl;

namespace {
void require(bool condition, std::string_view message) {
    if (!condition) throw std::runtime_error(std::string{message});
}
class simulated_motor : public device::motor_base {
public:
    simulated_motor() : motor_base{1s, 0.1f} { tick(); }
    awaitable<void> set(float target) override { linear_target = target; co_return; }
    awaitable<void> set_angle_speed(float target) override { angular_target = target; co_return; }
    awaitable<void> enable() override { set_enabled(true); co_return; }
    void set_enabled(bool next) override { enabled = next; if (!next) linear_target = angular_target = 0; }
    void measurements(float speed, float current = 0) { angle_speed_ = speed; torque_ = current; tick(); }
    void expire() { tick_time_ -= 2s; }
    bool enabled {};
    float linear_target {}, angular_target {};
};
void cycle(ctrl::shoot& shoot) {
    asio::io_context context;
    auto done = asio::co_spawn(context, shoot.update(), asio::use_future);
    context.run();
    done.get();
}
void request_fire(ctrl::shoot& shoot) {
    shoot.set_enabled(true);
    shoot.set_fire_permitted(true);
    shoot.set_friction_enabled(true);
    shoot.set_firing(true);
}
void send_frame(device::referee& referee, std::uint16_t command, std::vector<std::byte> payload) {
    referee.receive(device::referee_protocol::encode_frame(command, payload, 0));
}
}

void run_shoot_integration_tests() {
    simulated_motor left, right, trigger;
    left.measurements(-20); right.measurements(20); trigger.measurements(6);
    ctrl::shoot shoot;
    const ctrl::shoot::info_type info{.friction_params = {.acc = 1000}, .friction_max_speed = 2,
        .trigger_speed = 6, .control_time = 2ms};
    shoot.init(info, left, right, trigger);
    shoot.set_friction_enabled(true); shoot.set_firing(true); shoot.set_fire_permitted(true);
    cycle(shoot);
    require(!left.enabled && !right.enabled && !trigger.enabled && trigger.angular_target == 0 &&
        left.linear_target == 0 && right.linear_target == 0, "disabled shoot issued a nonzero command");
    request_fire(shoot);
    cycle(shoot);
    require(left.enabled && right.enabled && trigger.enabled && trigger.angular_target == 6 &&
        trigger.linear_target == 0 && left.linear_target == -2 && right.linear_target == 2,
        "shoot failed actual output chain or rad/s-vs-m/s dispatch");
    left.measurements(-15); right.measurements(15);
    cycle(shoot);
    require(shoot.friction_ready() && trigger.angular_target == 6,
        "exact positive readiness threshold must permit feed");
    left.measurements(0); right.measurements(0);
    cycle(shoot);
    require(!shoot.friction_ready() && trigger.angular_target == 0,
        "stationary friction wheels permitted feed");
    left.measurements(-20); right.measurements(20);
    shoot.set_friction_enabled(false);
    cycle(shoot);
    require(trigger.angular_target == 0, "coasting wheels bypassed friction switch");
    request_fire(shoot);
    shoot.set_fire_permitted(false);
    cycle(shoot);
    require(trigger.angular_target == 0 && right.linear_target == 2, "aim permission gate bypassed");
    shoot.set_fire_permitted(true);
    left.expire();
    cycle(shoot);
    require(trigger.angular_target == 0 && left.linear_target == 0 && right.linear_target == 0,
        "offline bound motor failed to stop all shoot outputs");
    left.measurements(-20);
    trigger.measurements(0, -5000);
    cycle(shoot);
    require(trigger.angular_target == 0, "reverse signed jam current bypassed hold");
    trigger.measurements(6, 0);
    cycle(shoot);
    require(trigger.angular_target == 0, "jam hold did not persist after current recovered");
    shoot.set_enabled(false);
    require(trigger.angular_target == 0 && !shoot.firing() && !shoot.friction_enabled(),
        "NoForce gate did not immediately clear pending firing state");
    request_fire(shoot);
    trigger.measurements(std::numeric_limits<float>::quiet_NaN());
    cycle(shoot);
    require(trigger.angular_target == 0, "NaN feedback permitted firing");
    trigger.measurements(6);
    right.measurements(std::numeric_limits<float>::infinity());
    cycle(shoot);
    require(trigger.angular_target == 0, "infinite friction speed counted as ready");
    shoot.set_enabled(false);

    for (const auto ready_speed : {0.0f, -1.0f, 3.0f}) {
        ctrl::shoot invalid;
        auto invalid_info = info;
        invalid_info.friction_ready_speed = ready_speed;
        bool rejected = false;
        try { invalid.init(invalid_info, left, right, trigger); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "nonpositive or unreachable friction threshold accepted");
    }
    ctrl::shoot stopped_friction;
    auto stopped_info = info;
    stopped_info.friction_max_speed = 0;
    bool rejected_stopped_friction = false;
    try { stopped_friction.init(stopped_info, left, right, trigger); }
    catch (const std::invalid_argument&) { rejected_stopped_friction = true; }
    require(rejected_stopped_friction, "zero friction operating speed accepted");

    auto& referee = get<device::referee>();
    referee.init({.offline_timeout = 1h});
    simulated_motor hero_left, hero_right, hero_trigger;
    hero_left.measurements(-20); hero_right.measurements(20); hero_trigger.measurements(-2);
    ctrl::shoot hero;
    auto hero_info = info;
    hero_info.enforce_referee = true; hero_info.bullet_caliber = 42; hero_info.trigger_speed = -2;
    hero.init(hero_info, hero_left, hero_right, hero_trigger);
    request_fire(hero);
    cycle(hero);
    require(hero_trigger.angular_target == 0, "missing referee state allowed feed");
    std::vector<std::byte> game(11); game[0] = std::byte{0x41};
    send_frame(referee, 0x0001, game);
    std::vector<std::byte> robot(13);
    robot[0] = std::byte{1}; robot[2] = std::byte{100}; robot[12] = std::byte{7};
    send_frame(referee, 0x0201, robot);
    std::vector<std::byte> ammo{std::byte{50}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}};
    send_frame(referee, 0x0208, ammo);
    cycle(hero);
    require(hero_trigger.angular_target == 0, "Hero incorrectly used 17 mm ammunition");
    ammo[2] = std::byte{1}; send_frame(referee, 0x0208, ammo);
    cycle(hero);
    require(hero_trigger.angular_target == -2, "valid 42 mm match gate blocked angular feed");
    game[0] = std::byte{0x31}; send_frame(referee, 0x0001, game);
    cycle(hero);
    require(hero_trigger.angular_target == 0, "non-battle match state permitted enforced feed");
    game[0] = std::byte{0x41}; send_frame(referee, 0x0001, game);
    robot[12] = std::byte{3}; send_frame(referee, 0x0201, robot);
    cycle(hero);
    require(hero_trigger.angular_target == 0, "referee shooter power off permitted feed");
    robot[12] = std::byte{7}; robot[2] = std::byte{0}; send_frame(referee, 0x0201, robot);
    cycle(hero);
    require(hero_trigger.angular_target == 0, "destroyed robot permitted feed");
    right.measurements(20); trigger.measurements(6);
    request_fire(shoot);
    cycle(shoot);
    hero.set_enabled(false);
    require(left.enabled && right.enabled && trigger.enabled && trigger.angular_target == 6,
        "secondary disable altered the primary shoot instance");
    require(!hero_left.enabled && !hero_right.enabled && !hero_trigger.enabled,
        "secondary shoot did not independently disable its motors");
}

#ifdef ROBOCTRL_SHOOT_TEST_MAIN
int main() { run_shoot_integration_tests(); }
#endif
