#pragma once
#include "device/motor/base.hpp"

namespace roboctrl::device{

class M9025 : public motor_base {
public:
    struct info_type{
        std::string name;
        std::string can_name;
    };
};

}