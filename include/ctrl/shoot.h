#pragma once
#include "utils/ramp.hpp"
#include "utils/singleton.hpp"
#include "core/async.hpp"
#include "core/logger.h"
#include "device/motor/ref.hpp"
#include <chrono>
#include <string_view>

namespace roboctrl::ctrl{
using namespace std::chrono_literals;

/**
* @brief 开火控制
* @details 负责拨弹和发射弹丸逻辑。
*/
class shoot : public utils::singleton_base<shoot>,public logable<shoot> {
public:
    shoot() = default;
    struct info_type{
        using owner_type = shoot;

        utils::ramp_f::params_type friction_params;
        float friction_max_speed {};
        float trigger_speed {};
        float friction_ready_speed {1.5f};
        float jam_current {4000.0f};
        float jam_speed {1.0f};
        std::chrono::steady_clock::duration jam_release_time {50ms};
        std::chrono::steady_clock::duration control_time {1ms};
        std::string_view left_friction_motor {"left_friction"};
        std::string_view right_friction_motor {"right_friction"};
        std::string_view trigger_motor {"trigger"};
    };

    inline std::string desc()const{return "shoot";}

    roboctrl::awaitable<void> task();

    bool init(const info_type& info);

    void set_firing(bool state);
    inline bool firing()const{return firing_;}
    void set_friction_enabled(bool state);
    inline bool friction_enabled() const{return friction_enabled_;}
    [[nodiscard]] bool friction_ready() const;

private:
    info_type info_;
    utils::ramp_f friction_ramp_;

    bool firing_ {false};
    bool friction_enabled_ {false};
    std::chrono::steady_clock::time_point jam_release_at_ {};
    device::motor_ref left_friction_motor_;
    device::motor_ref right_friction_motor_;
    device::motor_ref trigger_motor_;

};

static_assert(utils::singleton<shoot>);
}
