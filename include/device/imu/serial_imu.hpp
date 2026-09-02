#pragma once

#include <string>

#include "device/imu/base.hpp"

namespace roboctrl::device{
class serial_imu : public imu_base {
public:
    struct info_type{
        using key_type = std::string;
        using owner_type = serial_imu;

        std::string name;
        std::string serial_name;
        const std::string& key()const{
            return name;
        }
    };

    inline std::string desc()const{return std::format("serial_imu {} on serial {}",info_.name,info_.serial_name);}

    serial_imu(const info_type& info);
private:
    info_type info_;
};

static_assert(imu<serial_imu>);
};
