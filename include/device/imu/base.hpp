#pragma once

#include "device/base.hpp"
#include <array>
#include <span>

namespace roboctrl::device{ 
    
enum class axis : std::size_t {
    roll  = 0,
    pitch = 1,
    yaw   = 2,
    x = 0,
    y = 1,
    z = 2,
};

struct imu_base : public device_base {
protected:
    std::array<float, 3> acc_ {};
    std::array<float, 3> gyro_ {};
    std::array<float, 3> angle_ {};

public:
    auto acc() const { return std::span{ acc_ }; }
    auto gyro() const { return std::span{ gyro_ }; }
    auto angle() const { return std::span{ angle_ }; }
    float acc(const axis axis) const { return acc_[std::to_underlying(axis)]; }
    float gyro(const axis axis) const { return gyro_[std::to_underlying(axis)]; }
    float angle(const axis axis) const { return angle_[std::to_underlying(axis)]; }
};

template <typename T>
concept imu =
    std::derived_from<typename T::dev_type, imu_base>;
} // namespace dev