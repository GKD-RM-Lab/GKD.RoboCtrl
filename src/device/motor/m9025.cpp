#include "device/motor/m9025.h"
#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "io/base.hpp"
#include "io/can.h"
#include "utils/utils.hpp"
#include <cstddef>
#include <cstdint>

using namespace roboctrl;
using namespace roboctrl::device;

constexpr fp32 _rpm_to_rad_s = 2.f * Pi_f / 60.f;
constexpr fp32 _ecd_8192_to_rad  = 2.f * Pi_f / 8192.f;

M9025::M9025(const info_type& info):
    motor_base{1ms, info.radius},
    info_{info},
    pid_{info.pid_params}
{
    roboctrl::get<io::can>(info.can_name).on_data(0x140 + info.id,[&](const details::motor_upload_pkg& pkg) -> awaitable<void>{
        this->angle_ = _ecd_8192_to_rad * static_cast<float>(utils::make_u16(pkg.angle_h, pkg.angle_l));
        this->angle_speed_ = _rpm_to_rad_s * static_cast<float>(utils::make_i16(pkg.speed_h, pkg.speed_l));
        this->torque_ = utils::make_i16(pkg.current_h, pkg.speed_l);

        this->pid_.update(this->angle_speed_);

        std::array<std::byte,8> data;

        //TODO

        co_await roboctrl::get<io::can>(info_.can_name).send(0x200 + info_.id,data);

        this->log_debug("angle:{}, speed:{}, torque:{}",this->angle_,this->angle_speed_,this->torque_);
        this->tick();
    });
}

roboctrl::awaitable<void> M9025::set(fp32 speed)
{
    pid_.set_target(speed);
    pid_.update(this->angle_speed_);

    co_return;
}