#include "utils/ballistics.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace {
bool number(const char* text, double& value) {
    char* end {};
    value = std::strtod(text, &end);
    return end != text && *end == '\0' && std::isfinite(value);
}
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string_view{argv[1]} == "--help") {
        std::cout << "ballistics-cli x_m y_m z_m speed_m_s drag_1_s [vx_m_s vy_m_s vz_m_s]\n"
                     "Offline SI calculation. Pitch is positive up. Does not access hardware.\n";
        return 0;
    }
    if (argc != 6 && argc != 9) {
        std::cerr << "Use --help for arguments.\n";
        return 2;
    }
    double args[8] {};
    for (int i = 1; i < argc; ++i) {
        if (!number(argv[i], args[i - 1])) {
            std::cerr << "Invalid finite number: " << argv[i] << '\n';
            return 2;
        }
    }
    const roboctrl::utils::ballistic_target target {
        .position = {args[0], args[1], args[2]}, .velocity = {args[5], args[6], args[7]}};
    const auto result = roboctrl::utils::solve_ballistics({.drag = args[4]}, target, args[3]);
    if (!result) {
        const char* reason = "invalid input";
        switch (result.status) {
            case roboctrl::utils::ballistic_status::invalid_config: reason = "invalid config"; break;
            case roboctrl::utils::ballistic_status::no_intercept: reason = "no intercept bracketed in 5 s"; break;
            case roboctrl::utils::ballistic_status::not_converged: reason = "iteration limit"; break;
            default: break;
        }
        std::cerr << reason << '\n';
        return 1;
    }
    std::cout << std::setprecision(12)
              << "yaw_rad,pitch_up_rad,flight_time_s,target_x_m,target_y_m,target_z_m,residual_m\n"
              << result.yaw << ',' << result.pitch << ',' << result.flight_time << ','
              << result.target_position.x << ',' << result.target_position.y << ','
              << result.target_position.z << ',' << result.residual << '\n';
}
