#pragma once
#include <cstdint>

#include "core/async.hpp"
#include "device/base.hpp"
#include "utils/singleton.hpp"

namespace roboctrl::device{
/**
* @brief 超极电容
*/
class super_cap:public device_base,public utils::singleton_base<super_cap>,public logable<super_cap> {
public:
    struct info_type{
        using owner_type = super_cap;
        std::string can_name;
    };
    inline std::string desc()const{return "super cap";}

    bool init(const info_type& info);

    /**
     * @brief 设置超电状态
     * 
     * @param enable 是否启用
     * @param power_limit 功率限制
     */
    awaitable<void> set(bool enable,uint16_t power_limit);

    /**
     * @brief 当前底盘功率
     */
    inline float chassis_power()const{return chassis_power_;}
    /**
     * @brief 当前功率限制
     */
    inline uint16_t chassis_power_limit()const{return chassis_power_limit_;}

    /**
     * @brief 当前超电能量
     */
    inline uint8_t energy()const{return energy_;}
private:
    info_type info_;
    float chassis_power_;
    uint16_t chassis_power_limit_;
    uint8_t energy_;
};

static_assert(utils::singleton<super_cap>);


};