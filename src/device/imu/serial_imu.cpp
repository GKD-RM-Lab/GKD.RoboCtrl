#include "device/imu/serial_imu.hpp"
#include "device/imu/serial_imu_codec.hpp"
#include "io/serial.h"
#include "utils/utils.hpp"

#include <array>

using namespace roboctrl::device;

serial_imu::serial_imu(const info_type& info) : imu_base{100ms}, info_{info} {
    auto& serial = roboctrl::get<io::serial>(info.serial_name);
    serial.on_data(1, [this](const std::array<std::byte, 24>& bytes) {
        const auto packet = decode_serial_imu(bytes);
        if (!packet) return;
        const auto sample = convert_serial_imu(*packet,
            {info_.roll_sign, info_.pitch_sign, info_.yaw_sign},
            {info_.roll_rate_sign, info_.pitch_rate_sign, info_.yaw_rate_sign}, info_.gyro_scale);
        if (!sample) return;
        angle_ = sample->angle;
        gyro_ = sample->gyro;
        tick();
    });
}
