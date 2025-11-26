#include "ctrl/robot.h"
#include "core/async.hpp"
#include "ctrl/chassis.h"
#include "ctrl/gimbal.h"

using namespace roboctrl::ctrl;

bool robot::init(const info_type& info){
    roboctrl::init(
        info.chassis_info,
        info.gimbal_info
        //info.shoot_info
    );

    log_info("Robot initiated");
    
    return true;
}

roboctrl::awaitable<void> task(){
    co_return;
}