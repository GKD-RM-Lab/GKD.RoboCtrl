#include "ctrl/gimbal.h"
#include "core/async.hpp"

using namespace roboctrl::ctrl;
using namespace roboctrl;
using namespace roboctrl::device;

roboctrl::awaitable<void> gimbal::task(){
    while(true){
        
        co_await wait_for(1ms);
    }
}

bool gimbal::init(const info_type& info){
    yaw_motor_ = {info.yaw_motor_params.key, info.yaw_motor_params.controller_params};
    init_yaw_motor_ = {info.init_yaw_motor_params.key, info.init_yaw_motor_params.controller_params};
    pitch_motor_ = {info.pitch_motor_params.key, info.pitch_motor_params.controller_params};
    log_info("Gimbal initiated");
    roboctrl::spawn(task());
    return true;
}
