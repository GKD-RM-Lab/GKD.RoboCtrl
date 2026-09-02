/**
 * @file controlpad.h
 * @brief 遥控器串口设备与稳定输入快照。
 */
#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

#include "base.hpp"
#include "core/logger.h"
#include "utils/callback.hpp"

namespace roboctrl::device{

/** @brief 遥控器模拟通道的统一索引。 */
enum class control_channel : std::size_t { ch0 = 0, ch1, ch2, ch3, gimbal_pitch_wheel };
/** @brief 遥控器三段开关位置。 */
enum class switch_position : std::int32_t { unknown = 0, up = 1, down = 2, middle = 3 };

/** @brief 串口遥控输入的稳定内存表示。 */
struct control_pad_state {
    std::int32_t ch0 {};
    std::int32_t ch1 {};
    std::int32_t ch2 {};
    std::int32_t ch3 {};
    std::int32_t ch4 {};
    std::int32_t s1 {};
    std::int32_t s2 {};
    std::int32_t mouse_x {};
    std::int32_t mouse_y {};
    std::int32_t mouse_z {};
    std::int32_t mouse_l {};
    std::int32_t mouse_r {};
    std::int32_t key {};

    /** @brief 按抽象通道枚举读取通道值。 */
    std::int32_t channel(control_channel c) const noexcept {
        switch (c) {
        case control_channel::ch0: return ch0;
        case control_channel::ch1: return ch1;
        case control_channel::ch2: return ch2;
        case control_channel::ch3: return ch3;
        case control_channel::gimbal_pitch_wheel: return ch4;
        }
        return 0;
    }
    /** @brief 获取云台俯仰拨轮原始值。 */
    std::int32_t gimbal_pitch_wheel() const noexcept { return ch4; }
};

static_assert(sizeof(control_pad_state) == 52);

/**
 * @brief 遥控器设备，负责解析上报报文并发布输入状态
 * 
 */
class control_pad:public device_base,public logable<control_pad>{
public:
    /** @brief 遥控器实例化参数。 */
    struct info_type{
        std::string name;
        std::string serial_name;

        using key_type = std::string;
        using owner_type = control_pad;

        inline const std::string& key()const{return name;}
    };

    /** @brief 返回遥控器及其串口名称。 */
    inline std::string desc()const{
        return std::format("Control pad {} on serial:{}", info_.name, info_.serial_name);
    }

    /** @brief 构造遥控器设备并注册串口回调。 */
    control_pad(const info_type& info);

    /** @brief 获取最近一次有效报文解析出的输入快照。 */
    control_pad_state state() const noexcept { return state_; }

    /** @brief 获取云台俯仰拨轮原始值。 */
    fp32 pitch_wheel() const noexcept { return static_cast<fp32>(state_.ch4); }

    /** @brief 注册输入更新回调。 */
    template<callback_fn<control_pad_state> F>
    void on_update(F&& callback) {
        on_update_.add(std::forward<F>(callback));
    }
private:
    info_type info_;
    control_pad_state state_ {};
    roboctrl::callback<control_pad_state> on_update_;
};

static_assert(device<control_pad>);

}
