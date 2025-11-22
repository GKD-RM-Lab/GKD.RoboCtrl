#pragma once
#include "core/async.hpp"
#include "device/motor/base.hpp"
#include "utils/pid.h"

namespace roboctrl::device{

class M9025 : public motor_base,public logable<M9025> {
public:
    struct info_type{
        std::string name;
        std::string can_name;
        uint16_t id;

        utils::linear_pid::params_type pid_params;
        fp32 radius;
    };

    inline std::string desc() const{
        return std::format("M9025 motor {} on {}",info_.name,info_.can_name);
    }

    M9025(const info_type& info);
    awaitable<void> set(float speed);
    awaitable<void> enable();

private:
    info_type info_;
    utils::linear_pid pid_;
};

static_assert(motor<M9025>);

}