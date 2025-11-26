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
    uint8_t angle_h;
    uint8_t angle_l;
    uint8_t speed_h;
    uint8_t speed_l;
    uint8_t current_h;
    uint8_t current_l;
    uint8_t temperature;
    uint8_t unused;
} __attribute__((packed));

constexpr fp32 _rpm_to_rad_s = 2.f * Pi_f / 60.f;
constexpr fp32 _ecd_8192_to_rad  = 2.f * Pi_f / 8192.f;

static std::string __motor_tyep_to_string(dji_motor::type type){
    switch(type){
        case roboctrl::device::dji_motor::M2006:
            return "M2006";
        case roboctrl::device::dji_motor::M3508:
            return "M3508";
        case roboctrl::device::dji_motor::M6020:
            return "M6020";
        default:
            return "Unknown";
    }
}

dji_motor_group::dji_motor_group(dji_motor_group::info_type info):
    info_{info}
{
    std::fill(motors_.begin(),motors_.end(),nullptr);
    log_info("Dji Motor Group created on {}",info.can_name);
    roboctrl::spawn(task());
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

std::pair<uint16_t, uint16_t> dji_motor::can_pkg_id() const {
    switch(info_.type_) {
    case M2006:
    case M3508:
        if (info_.id >= 1 && info_.id <= 4)
            return {0x200, static_cast<uint16_t>(info_.id - 1)}; // index: 0..3
        else if (info_.id >= 5 && info_.id <= 8)
            return {0x1ff, static_cast<uint16_t>(info_.id - 5)}; // index: 0..3
        else {
            log_error("invalid dji motor id: {}", info_.id);
            return {0x200, 0};
        }
    case M6020:
        if (info_.id >= 1 && info_.id <= 4)
            return {0x1ff, static_cast<uint16_t>(info_.id - 1)};
        else if (info_.id >= 5 && info_.id <= 8)
            return {0x2ff, static_cast<uint16_t>(info_.id - 5)};
        else {
            log_error("invalid gm6020 id: {}", info_.id);
            return {0x1ff, 0};
        }
    }
}

roboctrl::awaitable<void> dji_motor_group::send_command(uint16_t can_id_) {
    std::array<std::byte,8> data{};
    bool flag = false;

    for (auto motor : motors_) {
        if (!motor) continue;

        auto [can_id, index] = motor->can_pkg_id();
        if (can_id == can_id_) {
            auto cur = motor->current();
            std::size_t offset = static_cast<std::size_t>(index) * 2;
            if (offset + 1 >= data.size()) {
                log_error("dji current index out of range: id={}, index={}", motor->info_.can_name, index);
                continue;
            }

            data[offset]     = utils::to_byte(static_cast<uint16_t>(cur) >> 8);
            data[offset + 1] = utils::to_byte(static_cast<uint16_t>(cur) & 0xff);
            flag = true;
        }
    }

    if (flag)
        co_await roboctrl::get<io::can>(info_.can_name).send(can_id_, data);
};

roboctrl::awaitable<void> dji_motor_group::task(){
    while(true){
        co_await send_command(0x1ff);
        co_await send_command(0x200);
        co_await send_command(0x2ff);

        co_await wait_for(1ms);
    }
}

dji_motor::dji_motor(dji_motor::info_type info)
    :info_{info},
    pid_{info.pid_params},
    motor_base{2ms,info.radius}
{
    auto& group = roboctrl::get(dji_motor_group::info_type::make(info.can_name));
    log_debug("Dji \"{}\" motor {} created on can \"{}\" with pid(p={},i={},d={},max iout={},max out={})",
        __motor_tyep_to_string(info.type_),
        info.name,
        info.can_name,
        info.pid_params.kp,
        info.pid_params.ki,
        info.pid_params.kd,
        info.pid_params.max_iout,
        info.pid_params.max_out
    );

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
        angle_ = _ecd_8192_to_rad * static_cast<float>(utils::make_u16(pkg.angle_h, pkg.angle_l)) ;
        angle_speed_ = _rpm_to_rad_s * static_cast<float>(utils::make_i16(pkg.speed_h, pkg.speed_l)) * reduction_ratio_;
        torque_ = utils::make_i16(pkg.current_h, pkg.current_l);

        pid_.update(linear_speed());
        current_ = pid_.state();

        log_debug("angle:{}, speed:{}, torque:{} ,linear speed:{},target speed:{}",this->angle_,this->angle_speed_,this->torque_,linear_speed(),pid_.target());
        tick();
    });

    group.register_motor(this);
    roboctrl::spawn(task());
}

roboctrl::awaitable<void> dji_motor::set(fp32 speed){ 
    pid_.set_target(speed);

    log_debug("target set to :{}",speed);

    co_return;
}

roboctrl::awaitable<void> dji_motor::task(){
    while(true){
        log_debug("pid output :{}",pid_.state());

        co_await wait_for(info_.control_time);
    }
}