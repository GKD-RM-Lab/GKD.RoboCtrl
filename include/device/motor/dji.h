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

    /** @brief 协议允许的最大设备 ID。未知型号返回 0。 */
    static constexpr int max_device_id(type motor_type) noexcept {
        switch (motor_type) {
        case M2006:
        case M3508:
            return 8;
        case M6020:
            return 7;
        }
        return 0;
    }

    /** @brief DJI 电调命令字段的型号级绝对值上限。未知型号返回 0。 */
    static constexpr fp32 command_current_limit(type motor_type) noexcept {
        switch (motor_type) {
        case M2006:
            return 10000.0f;
        case M3508:
            return 16384.0f;
        case M6020:
            return 30000.0f;
        }
        return 0.0f;
    }

    /**
     * @brief 电机初始化参数。
     */
    struct info_type{
        using key_type = std::string;
        using owner_type = dji_motor;

        type type_;
        int id;
        std::string name;
        std::string can_name;
        fp32 radius;
        utils::linear_pid::params_type pid_params;
        std::chrono::steady_clock::duration control_time;
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
     * @brief 设置目标电流或速度（取决于电调模式）。
     */
    awaitable<void> set(fp32 speed) override;
    awaitable<void> task();
    awaitable<void> enable() override { set_enabled(true); co_return; }
    void disable() override;
    void set_enabled(bool enabled) override;

    inline int16_t current() const {
        return enabled_ && target_fresh_since_enable_ && !offline()
            ? current_
            : int16_t{0};
    }
private: 
    std::pair<uint16_t,uint16_t> can_pkg_id() const;
private:
    friend dji_motor_group;
    info_type info_;
    int16_t current_ {0};
    fp32 reduction_ratio_ {1.0f};
    utils::linear_pid pid_;
    bool connected_ {false};
    bool started_ {false};
    bool enabled_ {false};
    bool target_fresh_since_enable_ {false};
    std::chrono::steady_clock::time_point last_feedback_at_ {};
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
     * @brief 立即按当前安全门状态刷新一轮全部命令帧。
     * @details 用于正常周期发送，也用于异常停机时在周期协程退出后显式补发零输出。
     */
    awaitable<void> flush_commands_once();

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
    bool started_ {false};
};

static_assert(multiton_info<dji_motor_group::info_type>);
static_assert(owner<dji_motor_group>);

}
