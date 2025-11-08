#pragma once

#include "core/task_context.hpp"
#include <string>
#include "ctrl/chassis.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{

class robot : public utils::singleton_base<robot>{
public:
    std::string desc()const{return "robot";}

    roboctrl::awaitable<void> task();
    inline void set_velocity(int x,int y){
        roboctrl::get<chassis>().set_velocity({x,y});
    }
    inline void set_velocity(vectori velocity){
        roboctrl::get<chassis>().set_velocity(velocity);
    }

    inline vectori velocity()const{return roboctrl::get<chassis>().velocity();}

    inline fp32 gimbal_yaw()const{return roboctrl::get<chassis>().gimbal_yaw();}
    inline void set_gimbal_yaw(fp32 yaw){roboctrl::get<chassis>().set_gimbal_yaw(yaw);}

    inline void set_chassis_rotate_speed(fp32 speed){roboctrl::get<chassis>().set_rotate_speed(speed);}
    inline fp32 chassis_rotate_speed()const{return roboctrl::get<chassis>().rotate_speed();}
private:
};
}