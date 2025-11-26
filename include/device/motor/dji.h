/**
 * @file dji.h
 * @brief DJI 电机及分组抽象。
 * @details 对常用 DJI 智能电调电机进行封装，提供 CAN 报文打包、分组发送能力。
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <string_view>

#include "base.hpp"
#include "core/logger.h"
#include "core/async.hpp"
#include "utils/pid.h"

namespace roboctrl::device{

class dji_motor_group;

/**
 * @brief DJI 系列电机。
 */
class dji_motor : public motor_base,public logable<dji_motor>{
public:
    enum type{
        M2006 = 2006,
        M3508 = 3508,
        M6020 = 6020
    };

    /**
     * @brief 电机初始化参数。
     */
    struct info_type{
        using key_type = std::string_view;
        using owner_type = dji_motor;

        type type_;
        int id;
        std::string_view name;
        std::string_view can_name;
        fp32 radius;
        utils::linear_pid::params_type pid_params;
        std::chrono::steady_clock::duration control_time;
        inline std::string_view key()const{return name;}
    };

    /**
     * @brief 描述信息，用于日志输出。
     */
    inline std::string desc() const{
        return std::format("Dji motor {}",info_.name);
    }

    dji_motor(info_type info);

    /**
     * @brief 设置目标电流或速度（取决于电调模式）。
     */
    awaitable<void> set(fp32 speed);
    awaitable<void> task();
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

/**
 * @brief 将同一 CAN 总线上的电机编组，一次性发送报文。
 */
class dji_motor_group : public logable<dji_motor_group>{
public:
    /**
     * @brief 分组初始化参数。
     */
    struct info_type{
        using key_type = std::string_view;
        using owner_type = dji_motor_group;

        std::string_view can_name;

        inline std::string_view key()const{return can_name;}

        static inline info_type make(std::string_view can_name){
            return{.can_name = can_name};
        }
    };

    /**
     * @brief 构造分组。
     */
    dji_motor_group(info_type info);

    /**
     * @brief 与调度器协同的任务，负责读取反馈等。
     */
    awaitable<void> task();

    /**
     * @brief 注册单个电机到分组内。
     */
    void register_motor(dji_motor* motor);
    inline std::string desc()const{return std::format("Dji motor group on can({})",info_.can_name);}

private:
    awaitable<void> send_command(uint16_t can_id_);
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
