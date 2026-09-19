#include "device/super_cap.h"
#include "core/async.hpp"
#include "io/can.h"
#include "utils/utils.hpp"

#include <cmath>

using namespace roboctrl::device;

namespace {

struct super_cap_receive_packet
{
    std::uint8_t error_code;
    float chassis_power;
    std::uint16_t chassis_power_limit;
    std::uint8_t cap_energy;
} __attribute__((packed));

static_assert(sizeof(super_cap_receive_packet) == 8);

} // namespace

bool super_cap::init(const super_cap::info_type& info){
    info_ = info;

    roboctrl::get<roboctrl::io::can>(info.can_name).on_data(0x51,[this](const super_cap_receive_packet& pkg){
        if (!std::isfinite(pkg.chassis_power)) {
            log_warn("discarding super-cap feedback with non-finite chassis power");
            return;
        }

        chassis_power_ = pkg.chassis_power;
        chassis_power_limit_ = pkg.chassis_power_limit;
        energy_ = pkg.cap_energy;
        tick();
        log_info("error code: {}, chassis power: {}, chassis power limit: {}, energy: {}",
                 pkg.error_code, chassis_power_, chassis_power_limit_, energy_);
    });

    return true;
}

roboctrl::awaitable<void> super_cap::set(bool enable,uint16_t power_limit)
{
    std::array<std::byte,8> data{};

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
