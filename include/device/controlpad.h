#pragma once
#include <cstdint>
#include <string_view>

#include "base.hpp"
#include "core/logger.h"
#include "utils/callback.hpp"

namespace roboctrl::device{

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
};

static_assert(sizeof(control_pad_state) == 52);

/**
 * @brief 遥控器设备，负责解析上报报文并发布输入状态
 * 
 */
class control_pad:public device_base,public logable<control_pad>{
public:
    struct info_type{
        std::string_view serial_name;

        using key_type = std::string_view;
        using owner_type = control_pad;

        inline std::string_view key()const{return serial_name;}
    };

    inline std::string desc()const{
        return std::format("Control pad on serial:{}",info_.serial_name);
    }

    control_pad(const info_type& info);

    template<callback_fn<control_pad_state> F>
    void on_update(F&& callback) {
        on_update_.add(std::forward<F>(callback));
    }
private:
    info_type info_;
    roboctrl::callback<control_pad_state> on_update_;
};

static_assert(device<control_pad>);

}
