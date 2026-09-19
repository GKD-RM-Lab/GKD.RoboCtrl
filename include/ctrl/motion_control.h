#pragma once

#include <chrono>
#include <string>

#include "core/async.hpp"
#include "ctrl/control_mapping.hpp"
#include "ctrl/motion_logic.hpp"
#include "device/chassis/base.hpp"
#include "device/controlpad.h"
#include "device/gimbal/base.hpp"
#include "utils/singleton.hpp"

namespace roboctrl::device { class aim_link; class navigation_link; class remote_logger; }
namespace roboctrl::ctrl {
using namespace std::chrono_literals;

class motion_control : public utils::singleton_base<motion_control>, public logable<motion_control> {
public:
    struct info_type {
        using owner_type = motion_control;
        std::string control_pad_key {"control_pad"};
        std::chrono::steady_clock::duration control_time {2ms};
        bool enable_shoot {false};
        utils::rad_pid::params_type follow_pid {};
        fp32 follow_direction {1.0f}, spin_recenter_speed {1.0f}, follow_tolerance {0.005f};
        bool enable_follow {true};
        std::string primary_aim_key, secondary_aim_key, navigation_key;
        bool enable_navigation {false}, search_when_vision_stale {false};
        fp32 search_yaw_speed {0.8f}, search_pitch_speed {5.0f};
        fp32 search_pitch_amplitude {0.2f}, search_pitch_center {0.165f};
        fp32 large_yaw_follow_direction {1.0f};
        std::string remote_logger_key;
        std::chrono::steady_clock::duration telemetry_period {50ms};
        bool red_team {true}; // Fallback only when no fresh referee identity exists.
    };
    bool init(const info_type& info);
    awaitable<void> task();
    std::string desc() const { return "motion control"; }
private:
    void dispatch(const control_command& command, fp32 dt);
    void stop_outputs();
    awaitable<void> send_telemetry();
    awaitable<void> send_debug_telemetry();
    info_type info_;
    device::control_pad_state input_ {};
    device::chassis_base* chassis_ {nullptr};
    device::gimbal_base* gimbal_ {nullptr};
    device::gimbal_base* secondary_gimbal_ {nullptr};
    device::gimbal_base* large_yaw_ {nullptr};
    device::aim_link* primary_aim_ {nullptr};
    device::aim_link* secondary_aim_ {nullptr};
    device::navigation_link* navigation_ {nullptr};
    device::remote_logger* remote_logger_ {nullptr};
    control_mapper mapper_ {};
    chassis_follow follow_;
    fp32 search_elapsed_ {};
    bool auto_aim_last_ {false};
};
static_assert(utils::singleton<motion_control>);
}
