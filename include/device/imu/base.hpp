/**
 * @file base.hpp
 * @brief IMU 抽象基类。
 * @details 统一惯导设备的姿态、加速度、角速度访问接口，便于上层算法使用。
 */
#pragma once

#include <cstddef>

#include "device/base.hpp"

namespace roboctrl::device{ 

struct euler_angle { fp32 roll{}, pitch{}, yaw{}; };
struct three_axis { fp32 x{}, y{}, z{}; };
    
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
    three_axis acc_ {};
    three_axis gyro_ {};
    euler_angle angle_ {};

public:
    /** @brief 获取三轴加速度。(rad/s^2) */
    three_axis acceleration() const { return acc_; }
    three_axis acc() const { return acc_; }
    /** @brief 获取三轴角速度。(rad/s)*/
    three_axis gyro() const { return gyro_; }
    /** @brief 获取欧拉角。(rad) */
    euler_angle angle() const { return angle_; }
    /** @brief 指定轴的加速度。 */
    fp32 acceleration(const axis a) const { return a == device::axis::x || a == device::axis::roll ? acc_.x : (a == device::axis::y || a == device::axis::pitch ? acc_.y : acc_.z); }
    fp32 acc(const axis a) const { return acceleration(a); }
    /** @brief 指定轴的角速度。 */
    fp32 gyro(const axis a) const { return a == device::axis::x || a == device::axis::roll ? gyro_.x : (a == device::axis::y || a == device::axis::pitch ? gyro_.y : gyro_.z); }
    /** @brief 指定轴的欧拉角。 */
    fp32 angle(const axis a) const { return a == device::axis::x || a == device::axis::roll ? angle_.roll : (a == device::axis::y || a == device::axis::pitch ? angle_.pitch : angle_.yaw); }

    inline explicit imu_base(const std::chrono::nanoseconds offline_timeout) : device_base{offline_timeout} {}
};

template <typename T>
concept imu =
    std::derived_from<T, imu_base>;
} // namespace dev
