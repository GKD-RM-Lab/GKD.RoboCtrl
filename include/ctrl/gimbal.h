#pragma once
#include "core/multiton.hpp"
#include "utils/pid.h"
#include "utils/singleton.hpp"
#include "utils/utils.hpp"

namespace roboctrl::ctrl{
class gimbal : public utils::singleton_base<gimbal>,public logable<gimbal>{
public:
    inline std::string desc()const{return "gimbal";}
    inline fp32 yaw()const{return yaw_;}
    inline void set_yaw(fp32 yaw){yaw_ = yaw;}

    struct info_type{
        using owner_type = gimbal;
    };

    bool init(const info_type& info);

private:
    fp32 yaw_ = 0;

    utils::rad_pid init_pid_;
    utils::rad_pid yaw_pid_;
    utils::rad_pid head_pid_;
};

static_assert(utils::singleton<gimbal>);
}