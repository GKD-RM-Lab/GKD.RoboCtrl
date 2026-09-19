#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <sys/types.h>

#include "device/motor/dji.h"
#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "io/base.hpp"
#include "io/can.h"
#include "utils/utils.hpp"

using namespace std::chrono_literals;
using namespace roboctrl::device;
using namespace roboctrl;

namespace {

struct dji_upload_pkg {
    uint8_t angle_h;
    uint8_t angle_l;
    uint8_t speed_h;
    uint8_t speed_l;
    uint8_t current_h;
    uint8_t current_l;
    uint8_t temperature;
    uint8_t unused;
} __attribute__((packed));

static_assert(sizeof(dji_upload_pkg) == 8);

struct dji_motor_measure {
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
};

dji_motor_measure parse_dji_upload_pkg(io::byte_span data) {
    const auto pkg = utils::from_bytes<dji_upload_pkg>(data);
    return {
        .ecd = utils::make_u16(pkg.angle_h, pkg.angle_l),
        .speed_rpm = utils::make_i16(pkg.speed_h, pkg.speed_l),
        .given_current = utils::make_i16(pkg.current_h, pkg.current_l),
        .temperature = pkg.temperature,
    };
}

int16_t saturate_current(fp32 value, fp32 limit) noexcept {
    if (!std::isfinite(value)) {
        return 0;
    }
    const auto safe_limit = std::clamp(
        limit,
        0.0f,
        static_cast<fp32>(std::numeric_limits<int16_t>::max()));
    return static_cast<int16_t>(std::clamp(value, -safe_limit, safe_limit));
}

constexpr fp32 _rpm_to_rad_s = 2.f * Pi_f / 60.f;
constexpr fp32 _ecd_8192_to_rad  = 2.f * Pi_f / 8192.f;

static std::string __motor_tyep_to_string(dji_motor::type type){
    switch(type){
        case roboctrl::device::dji_motor::M2006:
            return "M2006";
        case roboctrl::device::dji_motor::M3508:
            return "M3508";
        case roboctrl::device::dji_motor::M6020:
            return "M6020";
        default:
            return "Unknown";
    }
}

} // namespace

dji_motor_group::dji_motor_group(dji_motor_group::info_type info):
    info_{info}
{
    log_info("Dji Motor Group created on {}",info.can_name);
}

void dji_motor_group::start() {
    if (started_) {
        return;
    }
    started_ = true;
    roboctrl::spawn(task());
}

void dji_motor_group::register_motor(dji_motor* motor){
    for(auto m : motors_){
        if(m->can_pkg_id() == motor->can_pkg_id()){
            throw std::runtime_error(std::format(
                "DJI command slot conflict between {} and {}", m->desc(), motor->desc()));
        }
    }

    motors_.push_back(motor);
}

std::pair<uint16_t, uint16_t> dji_motor::can_pkg_id() const {
    switch(info_.type_) {
    case M2006:
    case M3508:
        if (info_.id >= 1 && info_.id <= 4)
            return {0x200, static_cast<uint16_t>(info_.id - 1)}; // index: 0..3
        else if (info_.id >= 5 && info_.id <= 8)
            return {0x1ff, static_cast<uint16_t>(info_.id - 5)}; // index: 0..3
        else {
            log_error("invalid dji motor id: {}", info_.id);
            return {0x200, 0};
        }
    case M6020:
        if (info_.id >= 1 && info_.id <= 4)
            return {0x1ff, static_cast<uint16_t>(info_.id - 1)};
        else if (info_.id >= 5 && info_.id <= 7)
            return {0x2ff, static_cast<uint16_t>(info_.id - 5)};
        else {
            log_error("invalid gm6020 id: {}", info_.id);
            return {0x1ff, 0};
        }
    }
    throw std::invalid_argument("unsupported DJI motor type");
}

roboctrl::awaitable<void> dji_motor_group::send_command(uint16_t can_id_) {
    std::array<std::byte,8> data{};
    bool flag = false;

    for (auto motor : motors_) {
        if (!motor) continue;

        auto [can_id, index] = motor->can_pkg_id();
        if (can_id == can_id_) {
            auto cur = motor->current();
            std::size_t offset = static_cast<std::size_t>(index) * 2;
            if (offset + 1 >= data.size()) {
                log_error("dji current index out of range: id={}, index={}", motor->info_.can_name, index);
                continue;
            }

            data[offset]     = utils::to_byte(static_cast<uint16_t>(cur) >> 8);
            data[offset + 1] = utils::to_byte(static_cast<uint16_t>(cur) & 0xff);
            flag = true;
        }
    }

    if (flag)
        co_await roboctrl::get<io::can>(info_.can_name).send(can_id_, data);
};

roboctrl::awaitable<void> dji_motor_group::task(){
    while(true){
        co_await flush_commands_once();

        co_await wait_for(1ms);
    }
}

roboctrl::awaitable<void> dji_motor_group::flush_commands_once(){
    co_await send_command(0x1ff);
    co_await send_command(0x200);
    co_await send_command(0x2ff);
}

dji_motor::dji_motor(dji_motor::info_type info)
    :info_{info},
    pid_{info.pid_params},
    motor_base{2ms,info.radius}
{
    if (info.name.empty()) {
        throw std::invalid_argument("DJI motor name must not be empty");
    }
    if (info.can_name.empty()) {
        throw std::invalid_argument(std::format("DJI motor {} has no CAN dependency", info.name));
    }
    if (!std::isfinite(info.radius) || info.radius <= 0.0f) {
        throw std::invalid_argument(std::format("DJI motor {} has invalid radius", info.name));
    }
    if (info.control_time <= std::chrono::steady_clock::duration::zero()) {
        throw std::invalid_argument(std::format("DJI motor {} has invalid control period", info.name));
    }
    const auto& pid = info.pid_params;
    if (!std::isfinite(pid.kp) || !std::isfinite(pid.ki) || !std::isfinite(pid.kd) ||
        !std::isfinite(pid.max_out) || !std::isfinite(pid.max_iout) ||
        pid.max_out < 0.0f || pid.max_iout < 0.0f) {
        throw std::invalid_argument(std::format("DJI motor {} has invalid PID parameters", info.name));
    }

    switch(info_.type_){
        case dji_motor::M2006:
            reduction_ratio_ = 1.f / 36.f;
            break;
        case dji_motor::M3508:
            reduction_ratio_ = 1.f / 19.f;
            break;
        case dji_motor::M6020:
            reduction_ratio_ = 1.f;
            break;
        default:
            throw std::invalid_argument(std::format(
                "DJI motor {} has unsupported type {}", info.name,
                static_cast<int>(info.type_)));
    }

    if (info.id < 1 || info.id > max_device_id(info.type_)) {
        throw std::invalid_argument(std::format(
            "DJI motor {} has invalid id {} for type {}",
            info.name, info.id, __motor_tyep_to_string(info.type_)));
    }
    const auto current_limit = command_current_limit(info.type_);
    if (pid.max_out > current_limit || pid.max_iout > current_limit) {
        throw std::invalid_argument(std::format(
            "DJI motor {} PID limit exceeds {} command range",
            info.name, __motor_tyep_to_string(info.type_)));
    }

    log_debug("Dji \"{}\" motor {} created on can \"{}\" with pid(p={},i={},d={},max iout={},max out={})",
        __motor_tyep_to_string(info.type_),
        info.name,
        info.can_name,
        info.pid_params.kp,
        info.pid_params.ki,
        info.pid_params.kd,
        info.pid_params.max_iout,
        info.pid_params.max_out
    );

}

void dji_motor::connect() {
    if (connected_) {
        return;
    }

    auto& group = roboctrl::get(dji_motor_group::info_type::make(info_.can_name));
    auto& can = roboctrl::get<io::can>(info_.can_name);

    uint16_t fallback_canid;

    switch(info_.type_){
        case dji_motor::M2006:
            fallback_canid = 0x200 + info_.id;
            break;
        case dji_motor::M3508:
            fallback_canid = 0x200 + info_.id;
            break;
        case dji_motor::M6020:
            fallback_canid = 0x204 + info_.id;
            break;
        default:
            throw std::invalid_argument(std::format(
                "DJI motor {} has unsupported type {}", info_.name,
                static_cast<int>(info_.type_)));
    }

    can.on_data(fallback_canid, [this](io::byte_span data) {
        const auto measure = parse_dji_upload_pkg(data);
        angle_ = _ecd_8192_to_rad * static_cast<float>(measure.ecd);
        angle_speed_ = _rpm_to_rad_s * static_cast<float>(measure.speed_rpm) * reduction_ratio_;
        torque_ = measure.given_current;

        const auto now = std::chrono::steady_clock::now();
        if (!enabled_ || !target_fresh_since_enable_) {
            current_ = 0;
            pid_.clean();
            last_feedback_at_ = {};
            tick();
            return;
        }

        fp32 dt = 0.0f;
        if (last_feedback_at_ != std::chrono::steady_clock::time_point{}) {
            const auto elapsed = now - last_feedback_at_;
            if (elapsed > std::chrono::steady_clock::duration::zero() &&
                elapsed <= info_.control_time * 5) {
                dt = std::chrono::duration_cast<std::chrono::duration<fp32>>(elapsed).count();
            } else {
                const fp32 target = pid_.target();
                pid_.clean();
                pid_.set_target(target);
            }
        }
        last_feedback_at_ = now;
        pid_.update(linear_speed(), dt);
        current_ = saturate_current(
            pid_.state(), command_current_limit(info_.type_));

        log_debug("angle:{}, speed:{}, torque:{} ,linear speed:{},target speed:{}",this->angle_,this->angle_speed_,this->torque_,linear_speed(),pid_.target());
        tick();
    }, sizeof(dji_upload_pkg));

    group.register_motor(this);
    connected_ = true;
}

void dji_motor::start() {
    if (!connected_) {
        throw std::logic_error(std::format("DJI motor {} must be connected before start", info_.name));
    }
    if (started_) {
        return;
    }
    started_ = true;
    roboctrl::spawn(task());
}

void dji_motor::disable() {
    enabled_ = false;
    target_fresh_since_enable_ = false;
    current_ = 0;
    pid_.clean();
    last_feedback_at_ = {};
}

void dji_motor::set_enabled(bool enabled) {
    if (enabled) {
        if (roboctrl::async::shutdown_requested()) {
            disable();
            return;
        }
        if (!enabled_) {
            current_ = 0;
            target_fresh_since_enable_ = false;
            pid_.clean();
            last_feedback_at_ = {};
        }
        enabled_ = true;
    } else {
        disable();
    }
}

roboctrl::awaitable<void> dji_motor::set(fp32 speed){ 
    if (!std::isfinite(speed)) {
        log_error("Rejected non-finite target for DJI motor {}", info_.name);
        current_ = 0;
        target_fresh_since_enable_ = false;
        pid_.clean();
        co_return;
    }
    pid_.set_target(speed);
    target_fresh_since_enable_ = enabled_;

    log_debug("target set to :{}",speed);

    co_return;
}

roboctrl::awaitable<void> dji_motor::task(){
    while(true){
        log_debug("pid output :{}",pid_.state());

        co_await wait_for(info_.control_time);
    }
}
