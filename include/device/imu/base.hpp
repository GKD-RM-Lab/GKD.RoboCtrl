/**
 * @file base.hpp
 * @brief IMU 抽象基类。
 * @details 统一惯导设备的姿态、加速度、角速度访问接口，便于上层算法使用。
 */
#pragma once

#include <array>
#include <span>

#include "device/base.hpp"

namespace roboctrl::device{ 
    
/**
 * @brief IMU 三轴枚举，兼容姿态/加速度轴序。
 */
enum class axis : std::size_t {
    roll  = 0,
    pitch = 1,
    yaw   = 2,
    x = 0,
    y = 1,
    z = 2,
};

/**
 * @brief IMU 基类，封装常见数据通道。
 */
struct imu_base : public device_base {
protected:
    std::array<fp32, 3> acc_ {};
    std::array<fp32, 3> gyro_ {};
    std::array<fp32, 3> angle_ {};

public:
    /** @brief 获取三轴加速度。(rad/s^2) */
    auto acc() const { return std::span{ acc_ }; }
    /** @brief 获取三轴角速度。(rad/s)*/
    auto gyro() const { return std::span{ gyro_ }; }
    /** @brief 获取欧拉角。(rad) */
    auto angle() const { return std::span{ angle_ }; }
    /** @brief 指定轴的加速度。 */
    fp32 acc(const axis axis) const { return acc_[std::to_underlying(axis)]; }
    /** @brief 指定轴的角速度。 */
    fp32 gyro(const axis axis) const { return gyro_[std::to_underlying(axis)]; }
    /** @brief 指定轴的欧拉角。 */
    fp32 angle(const axis axis) const { return angle_[std::to_underlying(axis)]; }

    inline explicit imu_base(const std::chrono::nanoseconds offline_timeout) : device_base{offline_timeout} {}
};

template <typename T>
concept imu =
    std::derived_from<T, imu_base>;
} // namespace dev
