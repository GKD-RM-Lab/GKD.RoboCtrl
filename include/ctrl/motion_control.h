#pragma once

#include <chrono>
#include <string>

#include "core/async.hpp"
#include "ctrl/control_mapping.hpp"
#include "device/chassis/base.hpp"
#include "device/controlpad.h"
#include "device/gimbal/base.hpp"
#include "utils/singleton.hpp"

namespace roboctrl::ctrl {
using namespace std::chrono_literals;

class motion_control : public utils::singleton_base<motion_control>, public logable<motion_control> {
public:
    struct info_type {
        using owner_type = motion_control;
        std::string control_pad_key {"control_pad"};
        std::chrono::steady_clock::duration control_time {2ms};
        bool enable_shoot {false};
    };

    bool init(const info_type& info);
    awaitable<void> task();
    std::string desc() const { return "motion control"; }

private:
    void dispatch(const control_command& command);
    void stop_outputs();

    std::string control_pad_key_;
    std::chrono::steady_clock::duration control_time_ {2ms};
    device::control_pad_state input_ {};
    device::chassis_base* chassis_ {nullptr};
    device::gimbal_base* gimbal_ {nullptr};
    control_mapper mapper_ {};
    bool enable_shoot_ {false};
};

static_assert(utils::singleton<motion_control>);

} // namespace roboctrl::ctrl
