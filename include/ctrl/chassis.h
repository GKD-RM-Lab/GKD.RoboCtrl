#pragma once

#include "core/async.hpp"
#include "core/logger.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{
class chassis :public utils::singleton_base<chassis>, public logable<chassis> {
public:
    awaitable<void> task();
    std::string desc()const {return "chassis";}

    inline void set_gimbal_yaw(fp32 yaw){gimbal_yaw_ = yaw;}
    inline fp32 gimbal_yaw()const {return gimbal_yaw_;}

    inline void set_velocity(vectorf velocity){velocity_ = velocity;}
    inline vectorf velocity()const {return velocity_;}

    inline void set_rotate_speed(fp32 speed){rotate_speed_ = speed;}
    inline fp32 rotate_speed()const {return rotate_speed_;}

    struct info_type{
        using owner_type = chassis;
    };

    bool init(const info_type& info);
private:
    awaitable<void> speed_decomposition();

    vectorf velocity_;
    fp32 gimbal_yaw_ {};
    fp32 rotate_speed_ {};
    fp32 max_wheel_speed_ {2.5f};
};

static_assert(utils::singleton<chassis>);
}