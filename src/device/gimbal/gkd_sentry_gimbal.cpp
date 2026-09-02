#include "device/gimbal/gkd_sentry_gimbal.hpp"
#include "core/async.hpp"
#include "device/imu/base.hpp"
#include "device/imu/serial_imu.hpp"
#include "utils/utils.hpp"

using namespace roboctrl;
using namespace roboctrl::device;

ROBOCTRL_REGISTER_GIMBAL("device.gkd_sentry_gimbal.v1", roboctrl::device::gkd_sentry_gimbal);
ROBOCTRL_REGISTER_GIMBAL("device.standard_imu_2axis_gimbal.v1", roboctrl::device::gkd_sentry_gimbal);
ROBOCTRL_REGISTER_GIMBAL("ctrl.standard_imu_2axis_gimbal.v1", roboctrl::device::gkd_sentry_gimbal);

roboctrl::awaitable<void> gkd_sentry_gimbal::task(){
    while(true){
        auto& imu = roboctrl::get<serial_imu>(imu_key_);
        const bool unavailable = imu.offline() || yaw_motor_->offline() || pitch_motor_->offline();
        const fp32 measured_yaw = imu.angle(axis::yaw);
        const fp32 measured_pitch = imu.angle(axis::pitch);
        const fp32 dt = std::chrono::duration_cast<std::chrono::duration<fp32>>(control_time_).count();

        if (!targets_initialized_ && !unavailable) {
            yaw_ = measured_yaw;
            pitch_ = std::clamp(measured_pitch, pitch_min_, pitch_max_);
            if (!yaw_zero_initialized_) {
                yaw_zero_ = yaw_motor_->angle();
                yaw_zero_initialized_ = true;
            }
            targets_initialized_ = true;
        }

        if (unavailable) {
            targets_initialized_ = false;
        } else if (!enabled_) {
            yaw_ = measured_yaw;
            pitch_ = std::clamp(measured_pitch, pitch_min_, pitch_max_);
        }

        if (!enabled_ || unavailable) {
            yaw_angle_pid_.clean();
            pitch_angle_pid_.clean();
            co_await yaw_motor_->set(0.0f);
            co_await pitch_motor_->set(0.0f);
        } else {
            yaw_angle_pid_.set_target(yaw_);
            yaw_angle_pid_.update(measured_yaw, dt);
            pitch_angle_pid_.set_target(pitch_);
            pitch_angle_pid_.update(measured_pitch, dt);
            co_await yaw_motor_->set(yaw_direction_ * yaw_angle_pid_.state());
            co_await pitch_motor_->set(pitch_direction_ * pitch_angle_pid_.state());
        }

        co_await wait_for(control_time_);
    }
}

bool gkd_sentry_gimbal::init(const info_type& info){
    imu_key_ = info.imu_key;
    yaw_motor_ = &roboctrl::get<dji_motor>(info.yaw_motor_key);
    pitch_motor_ = &roboctrl::get<dji_motor>(info.pitch_motor_key);
    yaw_angle_pid_ = utils::rad_pid{info.yaw_angle_pid};
    pitch_angle_pid_ = utils::rad_pid{info.pitch_angle_pid};
    yaw_direction_ = info.yaw_direction;
    pitch_direction_ = info.pitch_direction;
    pitch_min_ = info.pitch_min;
    pitch_max_ = info.pitch_max;
    control_time_ = info.control_time;
    log_info("Gimbal initiated");
    roboctrl::spawn(task());
    return true;
}
