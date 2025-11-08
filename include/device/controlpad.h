#pragma once
#include "base.hpp"
#include "core/logger.h"
#include "core/task_context.hpp"
#include "utils/callback.hpp"
#include <string_view>

namespace roboctrl::device{

class control_pad:public device_base,public logable<control_pad>{
public:
    struct info_type{
        std::string_view serial_name;

        using key_type = std::string_view;
        using owner_type = control_pad;

        inline std::string_view key()const{return serial_name;}
    };

    inline std::string desc()const{
        return std::format("Control pad on serial:{}",info_.serial_name);
    }

    control_pad(const info_type& info);
private:
    info_type info_;
    //callback<typename Args>
};

static_assert(device<control_pad>);

}