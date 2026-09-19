#pragma once

#include <chrono>
#include "device/motor/base.hpp"
#include "device/motor/protocol.hpp"
#include "utils/pid.h"

namespace roboctrl::device {

class m9025 : public motor_base, public logable<m9025> {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = m9025;
        std::string name;
        std::string can_name;
        uint16_t id {};
        utils::linear_pid::params_type pid_params;
        fp32 radius {};
        std::chrono::steady_clock::duration control_time {std::chrono::milliseconds{1}};
        std::chrono::steady_clock::duration offline_timeout {std::chrono::milliseconds{100}};
        // Legacy configuration uses 2*pi/65535. Override for installed firmware.
        fp32 encoder_counts_per_turn {65535.f};
        // Explicit: legacy field name is RPM, but actual firmware units are unverified.
        fp32 speed_rad_per_count {0.f};
        int direction {1};
        const std::string& key() const { return name; }
    };

    static bool valid_configuration(const info_type& info) {
        const auto& pid = info.pid_params;
        return !info.name.empty() && !info.can_name.empty() && info.id >= 1 && info.id <= 32
            && std::isfinite(info.radius) && info.radius > 0.f
            && std::isfinite(info.encoder_counts_per_turn) && info.encoder_counts_per_turn >= 1.f
            && info.encoder_counts_per_turn <= 65536.f
            && std::isfinite(info.speed_rad_per_count) && info.speed_rad_per_count > 0.f
            && info.speed_rad_per_count <= std::numeric_limits<fp32>::max() / 32768.f
            && (info.direction == 1 || info.direction == -1)
            && info.control_time > std::chrono::steady_clock::duration::zero()
            && info.offline_timeout >= info.control_time
            && std::isfinite(pid.kp) && std::isfinite(pid.ki) && std::isfinite(pid.kd)
            && std::isfinite(pid.max_out) && pid.max_out > 0.f && pid.max_out <= 32767.f
            && std::isfinite(pid.max_iout) && pid.max_iout >= 0.f && pid.max_iout <= pid.max_out;
    }
    explicit m9025(const info_type& info);
    void connect();
    void start();
    std::string desc() const { return std::format("M9025 motor {} on {}", info_.name, info_.can_name); }
    awaitable<void> set(fp32 speed) override;
    awaitable<void> set_angle_speed(fp32 speed) override;
    awaitable<void> set_current(fp32 command) override;
    awaitable<void> enable() override { set_enabled(true); co_return; }
    void disable() override;
    void set_enabled(bool enabled) override;
    awaitable<void> task();
    awaitable<void> stop_output();
    bool supports_current_control() const override { return true; }
    fp32 requested_current() const override { return enabled_ && !offline() ? current_ : 0.f; }
    fp32 max_current() const override { return std::min(info_.pid_params.max_out, 32767.f); }
    fp32 target_angle_speed() const override { return pid_.target(); }
    void set_output_scale(fp32 scale) override { output_scale_ = scale; }
    void set_current_limit(fp32 limit) override { current_limit_ = limit; }
    int16_t current() const {
        return motor_protocol::gated_current(current_, output_scale_, current_limit_, enabled_, !offline());
    }

private:
    info_type info_;
    utils::linear_pid pid_;
    fp32 current_ {0.f};
    fp32 output_scale_ {1.f};
    fp32 current_limit_ {std::numeric_limits<fp32>::infinity()};
    bool direct_current_ {false};
    bool connected_ {false};
    bool started_ {false};
    bool enabled_ {false};
};

using M9025 = m9025; // Compatibility spelling, not a parallel driver.
static_assert(motor<m9025>);

} // namespace roboctrl::device
