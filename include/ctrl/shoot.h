/**
 * @file shoot.h
 * @brief 发射器和拨弹机构控制。
 */
#pragma once
#include "utils/ramp.hpp"
#include "utils/singleton.hpp"
#include "core/async.hpp"
#include "core/logger.h"
#include "device/motor/base.hpp"
#include <chrono>
#include <string>

namespace roboctrl::ctrl{
using namespace std::chrono_literals;

/**
* @brief 开火控制
* @details 负责拨弹和发射弹丸逻辑。
*/
class shoot : public utils::singleton_base<shoot>,public logable<shoot> {
public:
    shoot() = default;
    /** @brief 发射器初始化参数。 */
    struct info_type{
        using owner_type = shoot;

        utils::ramp_f::params_type friction_params;
        float friction_max_speed {};
        float trigger_speed {}; // Output shaft rad/s, matching legacy trigger PID feedback.
        float friction_ready_speed {1.5f};
        float jam_current {4000.0f};
        float jam_speed {1.0f};
        std::chrono::steady_clock::duration jam_release_time {50ms};
        std::chrono::steady_clock::duration control_time {1ms};
        std::string left_friction_motor {"left_friction"};
        std::string right_friction_motor {"right_friction"};
        std::string trigger_motor {"trigger"};
        bool enforce_referee {false};
        unsigned int bullet_caliber {17}; // 17 mm normally, 42 mm Hero.
    };

    inline std::string desc()const{return "shoot";}

    /** @brief 周期更新摩擦轮和拨弹电机输出。 */
    roboctrl::awaitable<void> task();

    /** @brief 绑定发射器所需电机并初始化内部状态。 */
    bool init(const info_type& info);
    bool init(const info_type& info, device::motor_base& left,
        device::motor_base& right, device::motor_base& trigger);
    void start();
    /** One nonblocking control cycle, also usable by hardware-free simulation. */
    roboctrl::awaitable<void> update(fp32 dt);
    /** Robot owns this gate; only these bound actuators are enabled. */
    void set_enabled(bool enabled);
    void set_fire_permitted(bool permitted) { fire_permitted_ = permitted; }
    [[nodiscard]] bool fire_allowed() const;

    /** @brief 请求开始或停止拨弹。 */
    void set_firing(bool state);
    inline bool firing()const{return firing_;}
    /** @brief 请求启用或停止摩擦轮。 */
    void set_friction_enabled(bool state);
    inline bool friction_enabled() const{return friction_enabled_;}
    /** @brief 判断摩擦轮是否达到可发射的准备速度。 */
    [[nodiscard]] bool friction_ready() const;

private:
    info_type info_;
    utils::ramp_f friction_ramp_;

    bool enabled_ {false};
    bool fire_permitted_ {false};
    bool initialized_ {false};
    bool started_ {false};
    bool firing_ {false};
    bool friction_enabled_ {false};
    std::chrono::steady_clock::time_point jam_release_at_ {};
    device::motor_base* left_friction_motor_ {nullptr};
    device::motor_base* right_friction_motor_ {nullptr};
    device::motor_base* trigger_motor_ {nullptr};

};

static_assert(utils::singleton<shoot>);
}
