#pragma once
#include <cmath>
#include <algorithm>

namespace roboctrl::utils{


using fp32 = float;

template<typename T, auto error_measurer>
struct pid_base {
    T kp = 0;
    T ki = 0;
    T kd = 0;

    T max_out{};
    T max_iout{};

    using input_type = T;
    using state_type = T;

    struct params{
        T kp = 0;
        T ki = 0;
        T kd = 0;

        T max_out{};
        T max_iout{};
    };

    pid_base() = default;
    pid_base(fp32 p, fp32 i, fp32 d, T max_o, T max_io)
        : kp(p), ki(i), kd(d), max_out(max_o), max_iout(max_io) {}

    pid_base(const params& params)
        : kp(params.kp), ki(params.ki), kd(params.kd),
        max_out(params.max_out), max_iout(params.max_iout) {}
    void set_target(T target) { target_ = target; }
    T update(T current) {
        auto error = error_measurer(current, target_);

        fp32 pout = kp * error;

        integral_ += ki * error;
        integral_ = std::clamp(integral_, -max_iout, max_iout);

        fp32 derivative = kd * (error - last_error_);
        last_error_ = error;
        
        auto out = pout + integral_ + derivative;
        out = std::clamp(out, -max_out, max_out);

        output_ = out;
        return out;
    }

    void clean() {
        integral_ = 0;
        last_error_ = 0;
        output_ = 0;
    }

    T state() const { return output_; }

private:
    T target_ = 0;
    fp32 integral_ = 0;
    fp32 last_error_ = 0;
    T output_ = 0;
};

namespace details{

constexpr auto linear_error = [](fp32 cur, fp32 target) {
    return target - cur;
};

constexpr auto rad_error = [](fp32 cur, fp32 target) {
    fp32 diff = target - cur;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
};

}

using linear_pid = pid_base<fp32, details::linear_error>;
using rad_pid    = pid_base<fp32, details::rad_error>;

}