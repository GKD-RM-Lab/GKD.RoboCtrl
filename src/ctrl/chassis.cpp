#include "ctrl/chassis.h"
#include "core/task_context.hpp"
#include "ctrl/gimbal.h"
#include "device/motor/base.hpp"
#include "device/motor/dji.h"
#include "utils/utils.hpp"

using namespace roboctrl::ctrl;
using namespace roboctrl::device;

roboctrl::awaitable<void> chassis::task()
{
    while(true){
        co_await speed_decomposition();
        co_await roboctrl::wait_for(1ms);
    }
}

roboctrl::awaitable<void> chassis::speed_decomposition(){
    fp32 sin_yaw, cos_yaw;
    sincosf(roboctrl::get<gimbal>().yaw(), &sin_yaw, &cos_yaw);

    fp32 vx =  cos_yaw * velocity_.x + sin_yaw * velocity_.y;
    fp32 vy = -sin_yaw * velocity_.x + cos_yaw * velocity_.y;
    fp32 wz = rotate_speed_;

    fp32 w_lf = vx - vy - wz;
    fp32 w_rf = vx + vy + wz;
    fp32 w_lr = vx + vy - wz;
    fp32 w_rr = vx - vy + wz;

    float max_w = std::max({w_lf,w_rf,w_lr,w_rr});
    max_w = std::max(max_w,-std::min({w_lf,w_rf,w_lr,w_rr}));

    fp32 factor = 1.0;

    if(max_w > max_wheel_speed_) // 超速时按比例缩放
        fp32 factor = max_wheel_speed_ / max_w;

    co_await set_motor<dji_motor>("left_front_motor", w_lf * factor);
    co_await set_motor<dji_motor>("right_front_motor", w_rf * factor);
    co_await set_motor<dji_motor>("left_rear_motor", w_lr * factor);
    co_await set_motor<dji_motor>("right_rear_motor", w_rr * factor);
}