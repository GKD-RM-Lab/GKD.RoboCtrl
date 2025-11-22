#include <chrono>
#include <cstddef>
#include <cstdint>
#include <sys/types.h>

#include "device/motor/dji.h"
#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "io/base.hpp"
#include "io/can.h"
#include "utils/utils.hpp"

using namespace std::chrono_literals;
using namespace roboctrl::device;
using namespace roboctrl;

struct _dji_upload_pkg {
    uint8_t angle_h     : 1;
    uint8_t angle_l     : 1;
    uint8_t speed_h     : 1;
    uint8_t speed_l     : 1;
    uint8_t current_h   : 1;
    uint8_t current_l   : 1;
    uint8_t temperature : 1;
    uint8_t unused      : 1;
} __attribute__((packed));

constexpr fp32 _rpm_to_rad_s = 2.f * Pi_f / 60.f;
constexpr fp32 _ecd_8192_to_rad  = 2.f * Pi_f / 8192.f;

dji_motor_group::dji_motor_group(dji_motor_group::info_type info):
    info_{info}
{
    std::fill(motors_.begin(),motors_.end(),nullptr);
}

void dji_motor_group::register_motor(dji_motor* motor){
    for(auto m : motors_){
        if(m->can_pkg_id() == motor->can_pkg_id()){
            log_error("motor id conflict:{} and {}",m->desc(),motor->desc());
            return;
        }
    }

    motors_.push_back(motor);
}

std::pair<uint16_t,uint16_t> dji_motor::can_pkg_id() const{
    switch(info_.type_){
        case M2006:
        case M3508:
            if(info_.id < 4)
                return {0x200,info_.id};
            else return {0x1ff,info_.id - 4};
        case M6020:
            if(info_.id < 4)
                return {0x1ff,info_.id};
            else return {0x2ff,info_.id - 4};
    }
}


roboctrl::awaitable<void> dji_motor_group::task(){
    auto& can = roboctrl::get<io::can>(info_.can_name);

    while(true){
        std::array<std::byte,8> data;

        auto send_command = [&](uint16_t can_id_) -> awaitable<void>{
            std::array<std::byte,8> data;
            bool flag = false;

            for(auto motor : motors_){
                auto [can_id,motor_id] = motor->can_pkg_id();
                
                if(can_id == can_id_){
                    auto cur = motor->current();
                    data[motor_id * 2] = utils::to_byte(cur >> 8);
                    data[motor_id * 2 + 1] = utils::to_byte(cur & 0xff);
                    flag = true;
                }
            }

            if(flag)
                co_await can.send(can_id_,data);
        };

        co_await send_command(0x1ff);
        co_await send_command(0x200);
        co_await send_command(0x2ff);
    }
}

dji_motor::dji_motor(dji_motor::info_type info)
    :info_{info},
    pid_{info.pid_params},
    motor_base{2ms,info.radius}
{
    auto& group = roboctrl::get(dji_motor_group::info_type::make(info.can_name));

    auto& can = roboctrl::get<io::can>(info_.can_name);

    uint16_t fallback_canid;

    switch(info.type_){
        case dji_motor::M2006:
            reduction_ratio_ = 1.f / 36.f;
            fallback_canid = 0x200 + info.id;
            break;
        case dji_motor::M3508:
            reduction_ratio_ = 1.f / 19.f;
            fallback_canid = 0x200 + info.id;
            break;
        case dji_motor::M6020:
            reduction_ratio_ = 1.f;
            fallback_canid = 0x204 + info.id;
            break;
    }

    can.on_data(fallback_canid,[this](const _dji_upload_pkg& pkg) -> void{
        angle_ = _ecd_8192_to_rad * static_cast<float>(utils::make_u16(pkg.angle_h, pkg.angle_l));
        angle_speed_ = _rpm_to_rad_s * static_cast<float>(utils::make_i16(pkg.speed_h, pkg.speed_l));
        torque_ = utils::make_i16(pkg.current_h, pkg.speed_l);

        pid_.update(this->angle_speed_);
        current_ = pid_.state();

        log_debug("angle:{}, speed:{}, torque:{}",this->angle_,this->angle_speed_,this->torque_);
        tick();
    });

    group.register_motor(this);
}

roboctrl::awaitable<void> dji_motor::set(int16_t speed){ 
    pid_.set_target(speed);
    pid_.update(this->angle_speed_);

    co_return;
}