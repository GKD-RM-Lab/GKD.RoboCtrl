#include "device/super_cap.h"
#include "core/async.hpp"
#include "io/can.h"
#include "utils/utils.hpp"

using namespace roboctrl::device;

struct __super_cap_recive_pkg
{
    uint8_t errorCode;
    float chassisPower;
    uint16_t chassisPowerlimit;
    uint8_t capEnergy;
} __attribute__((packed));

bool super_cap::init(const super_cap::info_type& info){
    info_ = info;

    roboctrl::get<roboctrl::io::can>(info.can_name).on_data(0x51,[&](const __super_cap_recive_pkg& pkg){
        chassis_power_ = pkg.chassisPower;
        chassis_power_limit_ = pkg.chassisPowerlimit;
        energy_ = pkg.capEnergy;
        log_info("error code : {},chassis_power: {}, chassis_power_limit: {}, energy: {}",pkg.errorCode,chassis_power_,chassis_power_limit_,energy_);
    });

    return true;
}

roboctrl::awaitable<void> super_cap::set(bool enable,uint16_t power_limit)
{
    std::array<std::byte,8> data;

    if(enable)
        data[0] = utils::to_byte(1);
    else 
        data[0] = utils::to_byte(0);

    data[1] = utils::to_byte(power_limit & 0xff);
    data[2] = utils::to_byte(power_limit >> 8);
    data[3] = utils::to_byte(50 & 0xff);
    data[4] = utils::to_byte(50 >> 8);

    co_await roboctrl::get<roboctrl::io::can>(info_.can_name).send(0x61,data);
}