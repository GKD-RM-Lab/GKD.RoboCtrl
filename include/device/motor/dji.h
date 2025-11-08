#pragma once

#include "base.hpp"
#include "core/logger.h"
#include "core/task_context.hpp"
#include "io/base.hpp"
#include "utils/pid.h"
#include <cstdint>
#include <string_view>

namespace roboctrl::device{

class dji_motor_group;

class dji_motor : public motor_base,public logable<dji_motor>{
public:
    enum type{
        M2006 = 2006,
        M3508 = 3508,
        M6020 = 6020
    };

    struct info_type{
        using key_type = std::string_view;
        using owner_type = dji_motor;

        type type_;
        int id;
        std::string_view name;
        std::string can_name;
        fp32 radius;
        utils::linear_pid::params pid_params;
        inline std::string_view key()const{return name;}
    };

    inline std::string desc() const{
        return std::format("Dji motor(id={}) {} on {}",info_.id,info_.name,info_.can_name);
    }

    inline awaitable<void> task(){co_return ;}

    dji_motor(info_type info);

    awaitable<void> set(int16_t speed);
    awaitable<void> enable(){co_return ;}

    inline int16_t current()const{return current_;}
private: 
    std::pair<uint16_t,uint16_t> can_pkg_id() const;
private:
    friend dji_motor_group;
    info_type info_;
    int16_t current_;
    fp32 reduction_ratio_;
    utils::linear_pid pid_;
};

static_assert(multiton_info<dji_motor::info_type>);
static_assert(motor<dji_motor>);

class dji_motor_group : public logable<dji_motor_group>{
public:
    struct info_type{
        using key_type = std::string;
        using owner_type = dji_motor_group;

        std::string can_name;

        inline std::string key()const{return can_name;}

        static inline info_type make(std::string can_name){
            return{.can_name = can_name};
        }
    };

    dji_motor_group(info_type info);

    awaitable<void> task();

    void register_motor(dji_motor* motor);
    inline std::string desc()const{return std::format("Dji motor group on can({})",info_.can_name);}
private:

    struct motor_info{
        dji_motor* motor;
        int id;
        dji_motor::type type;
    };

    std::vector<dji_motor*> motors_;
    info_type info_;
};

static_assert(multiton_info<dji_motor_group::info_type>);
static_assert(owner<dji_motor_group>);

}