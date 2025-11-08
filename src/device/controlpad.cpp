#include "device/controlpad.h"
#include "core/task_context.hpp"
#include "device/base.hpp"
#include "io/serial.h"

using namespace roboctrl::device;

struct _controlpad_recive_pkg
{
    int ch0;
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int s1;
    int s2;
    int mouse_x;
    int mouse_y;
    int mouse_z;
    int mouse_l;
    int mouse_r;
    int key;
} __attribute__((packed));

control_pad::control_pad(const control_pad::info_type& info)
    :info_{info},
    device_base{0ms}
{
    auto& serial = roboctrl::get<io::serial>(info.serial_name);
    serial.on_data<_controlpad_recive_pkg>([&](const _controlpad_recive_pkg& pkg)->roboctrl::awaitable<void>{
        log_info("{}-{}-{}-{}-{}", pkg.ch0,pkg.ch1,pkg.ch2,pkg.ch3,pkg.ch4);
        co_return;
    });
}