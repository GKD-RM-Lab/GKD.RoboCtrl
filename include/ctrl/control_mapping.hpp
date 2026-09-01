#pragma once

#include <algorithm>

#include "device/controlpad.h"
#include "utils/utils.hpp"

namespace roboctrl::ctrl {

struct control_command {
    vectorf velocity {};
    fp32 rotate_speed {};
    fp32 yaw_delta {};
    fp32 pitch_delta {};
    fp32 pitch_target {};
    bool use_pitch_target {false};
    bool friction_enabled {false};
    bool firing {false};
    bool auto_aim {false};
    bool arm_requested {false};
};

/**
 * @brief 将旧工程的键鼠/遥控映射转换为控制层语义。
 * @details 必须先完成双开关加滚轮的解锁手势，输出才可能为非零。
 */
class control_mapper {
public:
    control_command update(const device::control_pad_state& input) {
        constexpr std::int32_t key_d = 0x1;
        constexpr std::int32_t key_a = 0x2;
        constexpr std::int32_t key_s = 0x4;
        constexpr std::int32_t key_w = 0x8;
        constexpr std::int32_t key_r = 0x40;
        constexpr std::int32_t key_f = 0x80;
        constexpr std::int32_t s_up = 1;
        constexpr std::int32_t s_down = 2;
        constexpr std::int32_t roll_up = -660;
        constexpr std::int32_t roll_down = 660;
        constexpr fp32 rc_scale = 660.0f;

        control_command command;
        if (input.s1 == s_down && input.s2 == s_down && input.ch4 == roll_up) {
            armed_ = true;
            command.arm_requested = true;
        }

        if (input.key != 0) {
            keyboard_mode_ = true;
        }
        if (!armed_) {
            return command;
        }

        if (keyboard_mode_) {
            command.velocity.x = static_cast<fp32>(bool(input.key & key_d))
                - static_cast<fp32>(bool(input.key & key_a));
            command.velocity.y = static_cast<fp32>(bool(input.key & key_w))
                - static_cast<fp32>(bool(input.key & key_s));

            const bool rotate_pressed = input.key & key_r;
            if (rotate_pressed && !rotate_pressed_last_) {
                rotate_enabled_ = !rotate_enabled_;
            }
            rotate_pressed_last_ = rotate_pressed;

            const bool friction_pressed = input.key & key_f;
            if (friction_pressed && !friction_pressed_last_) {
                friction_enabled_ = !friction_enabled_;
            }
            friction_pressed_last_ = friction_pressed;

            command.rotate_speed = rotate_enabled_ ? 1.0f : 0.0f;
            command.friction_enabled = friction_enabled_;
            command.auto_aim = input.mouse_r || (input.s1 == s_down && input.s2 == s_up);
            command.firing = input.mouse_l || input.ch4 == roll_down;
            if (!command.auto_aim) {
                command.yaw_delta = static_cast<fp32>(input.mouse_x) / 10000.0f;
                command.pitch_delta = static_cast<fp32>(input.mouse_y) / 10000.0f;
            }
            return command;
        }

        command.velocity = {
            .x = static_cast<fp32>(input.ch3) / rc_scale * 3.0f,
            .y = static_cast<fp32>(input.ch2) / rc_scale * 3.0f
        };
        command.yaw_delta = static_cast<fp32>(input.ch0) / rc_scale / 200.0f;
        command.pitch_target = std::clamp(
            static_cast<fp32>(input.ch1) / rc_scale * 0.3f, -0.3f, 0.3f);
        command.use_pitch_target = true;
        command.rotate_speed = input.s1 == s_up ? 1.0f : 0.0f;
        command.friction_enabled = input.s2 == s_up;
        command.firing = input.ch4 == roll_down;
        friction_enabled_ = command.friction_enabled;
        return command;
    }

    [[nodiscard]] bool armed() const noexcept { return armed_; }

    void reset() noexcept {
        armed_ = false;
        keyboard_mode_ = false;
        rotate_enabled_ = false;
        friction_enabled_ = false;
        rotate_pressed_last_ = false;
        friction_pressed_last_ = false;
    }

private:
    bool armed_ {false};
    bool keyboard_mode_ {false};
    bool rotate_enabled_ {false};
    bool friction_enabled_ {false};
    bool rotate_pressed_last_ {false};
    bool friction_pressed_last_ {false};
};

} // namespace roboctrl::ctrl
