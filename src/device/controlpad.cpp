#include "device/controlpad.h"
#include "device/base.hpp"
#include "io/serial.h"

using namespace roboctrl::device;

control_pad::control_pad(const control_pad::info_type& info)
    :info_{info},
    device_base{100ms}
{
    auto& serial = roboctrl::get<io::serial>(info.serial_name);
    serial.on_data(2,[this](const control_pad_state& state){
        tick();
        on_update_(state);
    });
}
