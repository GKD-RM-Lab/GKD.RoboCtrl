#pragma once
#include "utils/ramp.hpp"
#include "utils/singleton.hpp"
#include "core/async.hpp"
#include "core/logger.h"

namespace roboctrl::ctrl{

/**
* @brief 开火控制
* @details 负责拨弹和发射弹丸逻辑。
*/
class shoot : public utils::singleton_base<shoot>,public logable<shoot> {
public:
    shoot() = default;
    struct info_type{
        using owner_type = shoot;

        utils::ramp_f::params_type friction_params;
        float friction_max_speed;
    };

    inline std::string desc()const{return "shoot";}

    roboctrl::awaitable<void> task();

    bool init(const info_type& info);

    void set_firing(bool state);
    inline bool firing()const{return firing_;}

private:
    info_type info_;
    utils::ramp_f friction_ramp_;

    bool firing_;

};

static_assert(utils::singleton<shoot>);
}