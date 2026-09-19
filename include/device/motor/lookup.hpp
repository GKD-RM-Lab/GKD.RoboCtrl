#pragma once

#include <string_view>
#include "device/motor/dji.h"
#include "device/motor/j6006.h"
#include "device/motor/m9025.h"

namespace roboctrl::device {

inline motor_base& find_motor(std::string_view type, const std::string& name) {
    if (type == "dji") return roboctrl::get<dji_motor>(name);
    if (type == "j6006") return roboctrl::get<j6006>(name);
    if (type == "m9025") return roboctrl::get<m9025>(name);
    throw std::invalid_argument(std::format("unknown motor type '{}' for {}", type, name));
}

} // namespace roboctrl::device
