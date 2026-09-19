#include "device/controlpad.h"
#include "device/base.hpp"
#include "io/serial.h"

#include <cstdint>
#include <limits>

using namespace roboctrl::device;

namespace {

bool valid_control_pad_state(const control_pad_state& state) noexcept {
    constexpr std::int32_t channel_limit = 660;
    const auto valid_channel = [](std::int32_t value) {
        return value >= -channel_limit && value <= channel_limit;
    };
    const auto valid_switch = [](std::int32_t value) {
        return value >= static_cast<std::int32_t>(switch_position::up) &&
            value <= static_cast<std::int32_t>(switch_position::middle);
    };
    const auto valid_mouse_axis = [](std::int32_t value) {
        return value >= std::numeric_limits<std::int16_t>::min() &&
            value <= std::numeric_limits<std::int16_t>::max();
    };

    return valid_channel(state.ch0) && valid_channel(state.ch1) &&
        valid_channel(state.ch2) && valid_channel(state.ch3) &&
        valid_channel(state.ch4) && valid_switch(state.s1) &&
        valid_switch(state.s2) && valid_mouse_axis(state.mouse_x) &&
        valid_mouse_axis(state.mouse_y) && valid_mouse_axis(state.mouse_z) &&
        (state.mouse_l == 0 || state.mouse_l == 1) &&
        (state.mouse_r == 0 || state.mouse_r == 1) &&
        state.key >= 0 && state.key <= std::numeric_limits<std::uint16_t>::max();
}

} // namespace

control_pad::control_pad(const control_pad::info_type& info)
    :info_{info},
    device_base{100ms}
{
    auto& serial = roboctrl::get<io::serial>(info.serial_name);
    serial.on_data(2,[this](const control_pad_state& state){
        if (!valid_control_pad_state(state)) {
            log_warn("Rejected invalid control-pad frame");
            return;
        }
        state_ = state;
        tick();
        on_update_(state);
    });
}
