#pragma once

#include "device/motor/base.hpp"
#include "device/motor/protocol.hpp"

namespace roboctrl::device {

class j6006 : public motor_base, public logable<j6006> {
public:
    struct info_type {
        using key_type = std::string;
        using owner_type = j6006;
        std::string name;
        std::string can_name;
        uint16_t id {};
        uint16_t master_id {0};
        fp32 radius {1.f};
        motor_protocol::j6006_ranges feedback_range {};
        fp32 max_speed {45.f};
        int direction {1};
        std::chrono::steady_clock::duration control_time {std::chrono::milliseconds{1}};
        std::chrono::steady_clock::duration offline_timeout {std::chrono::milliseconds{100}};
        const std::string& key() const { return name; }
    };

    static bool valid_configuration(const info_type& info) {
        return !info.name.empty() && !info.can_name.empty() && info.id >= 1 && info.id <= 15
            && info.master_id <= 0x7ff && std::isfinite(info.radius) && info.radius > 0.f
            && motor_protocol::valid_ranges(info.feedback_range)
            && std::isfinite(info.max_speed) && info.max_speed > 0.f
            && info.max_speed <= info.feedback_range.velocity_max
            && (info.direction == 1 || info.direction == -1)
            && info.control_time > std::chrono::steady_clock::duration::zero()
            && info.offline_timeout >= info.control_time;
    }
    explicit j6006(const info_type& info);
    void connect();
    void start();
    std::string desc() const { return std::format("J6006 motor {} on {}", info_.name, info_.can_name); }
    awaitable<void> set(fp32 linear_speed) override;
    awaitable<void> set_angle_speed(fp32 speed) override;
    awaitable<void> enable() override { set_enabled(true); co_return; }
    void disable() override;
    void set_enabled(bool enabled) override;
    awaitable<void> task();
    awaitable<void> stop_output();
    fp32 target_angle_speed() const override { return target_speed_; }
    uint16_t encoder_raw() const { return encoder_raw_; }
    uint8_t status() const { return status_; }
    bool faulted() const override { return fault_latched_ || (status_ >= 8 && status_ <= 14); }
    fp32 command_speed() const {
        return motor_protocol::gated_j6006_velocity(target_speed_, enabled_ && !fault_latched_, !offline(), status_);
    }
    fp32 current_feedback_raw() const override { return 0.f; } // Feedback is torque Nm.

private:
    info_type info_;
    fp32 target_speed_ {0.f};
    uint16_t encoder_raw_ {0};
    uint8_t status_ {0xff};
    bool connected_ {false};
    bool started_ {false};
    bool enabled_ {false};
    bool previous_enabled_ {false};
    bool fault_latched_ {false};
};

static_assert(motor<j6006>);

} // namespace roboctrl::device
