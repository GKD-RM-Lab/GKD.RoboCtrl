#pragma once

#include <string>

#include "core/async.hpp"
#include "ctrl/chassis.h"
#include "ctrl/gimbal.h"
#include "ctrl/shoot.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{

enum class robot_state{
    NoForce,
    FinishInit,
    FollowGimbal,
    Search,
    Idle,
    NotFollow
};

class robot : public utils::singleton_base<robot>{
public:
    robot() = default;
    struct info_type{
        using owner_type = robot;
        gimbal::info_type gimbal_info;
        chassis::info_type chassis_info;
        shoot::info_type shoot_info;
    };

    bool init(const info_type& info);
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

    robot_state state()const{return state_;}
    void set_state(robot_state state){state_ = state;}

    
private:
    robot_state state_ {robot_state::Idle};
};

static_assert(utils::singleton<robot>);
}