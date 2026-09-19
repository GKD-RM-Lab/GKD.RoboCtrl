#pragma once

#include <cstdint>
#include "core/async.hpp"
#include "device/base.hpp"
#include "utils/singleton.hpp"

namespace roboctrl::device {

class super_cap : public device_base, public utils::singleton_base<super_cap>, public logable<super_cap> {
public:
    struct info_type {
        using owner_type = super_cap;
        std::string can_name;
        uint16_t receive_id {0x51};
        uint16_t command_id {0x61};
        uint16_t max_power_limit {150};
        uint16_t buffer_target {50};
        std::chrono::steady_clock::duration resend_time {std::chrono::milliseconds{50}};
        std::chrono::steady_clock::duration command_timeout {std::chrono::milliseconds{100}};
    };

    super_cap() : device_base{std::chrono::milliseconds{500}} {}
    static bool valid_configuration(const info_type& info) {
        return !info.can_name.empty() && info.receive_id <= 0x7ff && info.command_id <= 0x7ff
            && info.receive_id != info.command_id && info.max_power_limit > 0
            && info.resend_time > std::chrono::steady_clock::duration::zero()
            && info.command_timeout >= info.resend_time;
    }
    std::string desc() const { return "super cap"; }
    bool init(const info_type& info);
    void connect();
    void start();
    awaitable<void> task();
    awaitable<void> set(bool enable, uint16_t power_limit);
    bool configured() const { return configured_; }
    float chassis_power() const { return chassis_power_; }
    uint16_t chassis_power_limit() const { return chassis_power_limit_; }
    uint8_t energy() const { return energy_; }
    uint8_t error_code() const { return error_code_; }
    uint64_t sample_sequence() const { return sample_sequence_; }

private:
    info_type info_;
    float chassis_power_ {0.f};
    uint16_t chassis_power_limit_ {0};
    uint8_t energy_ {0};
    uint8_t error_code_ {0};
    uint64_t sample_sequence_ {0};
    uint16_t requested_power_limit_ {0};
    bool requested_enabled_ {false};
    bool configured_ {false};
    bool connected_ {false};
    bool started_ {false};
    std::chrono::steady_clock::time_point last_command_ {};
};

static_assert(utils::singleton<super_cap>);

} // namespace roboctrl::device
