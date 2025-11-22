#include "ctrl/robot.h"

using namespace roboctrl::ctrl;

bool robot::init(const info_type& info){
    roboctrl::init(
        info.chassis_info,
        info.gimbal_info,
        info.shoot_info
    );

    return true;
}