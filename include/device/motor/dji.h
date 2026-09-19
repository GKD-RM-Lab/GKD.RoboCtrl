/**
 * @file dji.h
 * @brief DJI 电机及分组抽象。
 * @details 对常用 DJI 智能电调电机进行封装，提供 CAN 报文打包、分组发送能力。
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "base.hpp"
#include "core/logger.h"
#include "core/async.hpp"
#include "utils/pid.h"
#include "protocol.hpp"

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
        using key_type = std::string;
        using owner_type = dji_motor;

        type type_ {};
        int id {};
        std::string name;
        std::string can_name;
        fp32 radius {};
        utils::linear_pid::params_type pid_params;
        std::chrono::steady_clock::duration control_time {};
        inline const std::string& key()const{return name;}
    };

    /**
     * @brief 描述信息，用于日志输出。
     */
    inline std::string desc() const{
        return std::format("Dji motor {}",info_.name);
    }

    dji_motor(info_type info);

    void connect();
    void start();

    /**
     * @brief 设置线速度目标，单位 m/s；角速度与原始电流使用显式接口。
     */
    awaitable<void> set(fp32 speed) override;
    awaitable<void> set_angle_speed(fp32 speed) override;
    awaitable<void> set_current(fp32 command) override;
    bool supports_current_control() const override { return true; }
    fp32 requested_current() const override { return enabled_ && !offline() ? current_ : 0.f; }
    fp32 max_current() const override { return std::min(info_.pid_params.max_out, 32767.f); }
    fp32 target_angle_speed() const override { return mode_ == control_mode::linear ? pid_.target() / radius_ : pid_.target(); }
    void set_output_scale(fp32 scale) override { output_scale_ = scale; }
    void set_current_limit(fp32 limit) override { current_limit_ = limit; }
    awaitable<void> task();
    awaitable<void> enable() override { enabled_ = true; co_return; }
    void disable() override;
    void set_enabled(bool enabled) override;

    inline int16_t current() const {
        return motor_protocol::gated_current(current_, output_scale_, current_limit_, enabled_, !offline());
    }
private: 
    std::pair<uint16_t,uint16_t> can_pkg_id() const;
private:
    friend dji_motor_group;
    info_type info_;
    enum class control_mode { linear, angular, current };
    control_mode mode_ {control_mode::linear};
    fp32 current_ {0.f};
    fp32 output_scale_ {1.f};
    fp32 current_limit_ {std::numeric_limits<fp32>::infinity()};
    fp32 reduction_ratio_ {1.0f};
    utils::linear_pid pid_;
    bool connected_ {false};
    bool started_ {false};
    bool enabled_ {false};
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
        using key_type = std::string;
        using owner_type = dji_motor_group;

        std::string can_name;

        inline const std::string& key()const{return can_name;}

        static inline info_type make(std::string can_name){
            return{.can_name = can_name};
        }
    };

    /**
     * @brief 构造分组。
     */
    dji_motor_group(info_type info);

    void start();

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
        int id {};
        dji_motor::type type;
    };

    std::vector<dji_motor*> motors_;
    info_type info_;
    bool started_ {false};
};

static_assert(multiton_info<dji_motor_group::info_type>);
static_assert(owner<dji_motor_group>);

}
