#include "device/imu/serial_imu.hpp"
#include "device/imu/base.hpp"
#include "io/serial.h"
#include "utils/utils.hpp"

#include <cmath>

using namespace roboctrl::device;

struct __serial_imu_pkg
{
    float yaw;
    float pitch;
    float roll;
    float yaw_v;
    float pitch_v;
    float roll_v;
} __attribute__((packed));

serial_imu::serial_imu(const info_type& info) :imu_base{100ms}, info_{info} {
    auto& serial = roboctrl::get<io::serial>(info.serial_name);
    serial.on_data(1,[&](const __serial_imu_pkg& pkg){
        if (!std::isfinite(pkg.yaw) || !std::isfinite(pkg.pitch) ||
            !std::isfinite(pkg.roll) || !std::isfinite(pkg.yaw_v) ||
            !std::isfinite(pkg.pitch_v) || !std::isfinite(pkg.roll_v)) {
            LOG_WARN("Rejected non-finite serial IMU frame");
            return;
        }
        angle_ = {
            utils::rad_format(pkg.roll * Pi_f / 180.f), 
            utils::rad_format(pkg.pitch * Pi_f / 180.f),
            utils::rad_format(pkg.yaw * Pi_f / 180.f)
        };
        gyro_ = {
            pkg.roll_v * (Pi_f / 180.f) / 1000.f, 
            pkg.pitch_v * (Pi_f / 180.f) / 1000.f,
            pkg.yaw_v * (Pi_f / 180.f) / 1000.f
        };

        tick();
    });
}
