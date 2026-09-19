#include "config/runtime.hpp"

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
}

void run_configuration_migration_tests() {
    using namespace roboctrl;
    auto root = std::filesystem::current_path();
    while (!std::filesystem::exists(root / "configs/infantry.yaml")) {
        require(root.parent_path() != root, "configuration test repository root missing");
        root = root.parent_path();
    }
    auto loaded = config::load_configuration(root / "configs/infantry.yaml");
    require(bool(loaded), loaded ? "" : loaded.error().c_str());
    const auto good = *loaded;
    const auto reject = [&](const auto& mutate) {
        auto item = good;
        mutate(item);
        require(!config::validate_runtime_configuration(std::move(item)), "invalid migration configuration accepted");
    };
    reject([](auto& c) { c.additional_imus.push_back(c.imu); });
    reject([](auto& c) { auto alias = c.cans.front(); alias.name = "alias"; c.cans.push_back(alias); });
    reject([](auto& c) { c.robot.gimbal_info.yaw_motor_type = "j6006"; });
    reject([](auto& c) { c.robot.secondary_gimbal_info = c.robot.gimbal_info; });
    reject([](auto& c) { c.robot.gimbal_info.yaw_angle_direction = 0; });
    reject([](auto& c) { c.robot.chassis_info.wheel_directions[0] = 0; });
    reject([](auto& c) { c.imu.pitch_sign = std::numeric_limits<float>::quiet_NaN(); });
    reject([](auto& c) { c.aim_links[0].target_timeout_ms = 0; });
    reject([](auto& c) { c.aim_links[0].header = 0x37; });
    reject([](auto& c) { c.aim_links[0].address = "invalid-address"; });
    reject([](auto& c) { c.aim_links[0].udp_name = "missing"; });
    reject([](auto& c) { auto link = c.aim_links[0]; link.key_ = "duplicate-route"; c.aim_links.push_back(link); });
    reject([](auto& c) { auto server = c.udp_servers[0]; server.key_ = "duplicate-port"; c.udp_servers.push_back(server); });
    reject([](auto& c) { c.robot.motion_info.remote_logger_key = "missing"; });
    reject([](auto& c) { c.referee.reset(); });
    reject([](auto& c) { c.referee->serial_name = c.imu.serial_name; });
    reject([](auto& c) { c.serials.back().device = c.serials.front().device; });
    reject([](auto& c) { c.robot.shoot_info.friction_ready_speed = 0; });
    reject([](auto& c) { c.robot.shoot_info.friction_ready_speed = c.robot.shoot_info.friction_max_speed + 1; });
    reject([](auto& c) { c.super_cap->command_id = 0x200; });
    reject([](auto& c) { c.power->configured_power_limit = -1; });
    reject([](auto& c) { c.robot.enable_chassis = false; });
    reject([](auto& c) { c.robot.motion_info.enable_navigation = true; });
    require(good.robot.shoot_info.trigger_speed == 6.f, "trigger rad/s target changed unexpectedly");
    require(good.robot.gimbal_info.yaw_rate_pid.ki == 234000.f, "legacy 1ms PID I conversion missing");
    require(good.robot.gimbal_info.yaw_rate_pid.kd == .2f, "legacy 1ms PID D conversion missing");
    auto example = config::load_configuration(root / "configs/examples/m9025-bench.yaml");
    require(bool(example), example ? "" : example.error().c_str());
    auto oversized = *example;
    oversized.m9025_motors[0].speed_rad_per_count = std::numeric_limits<float>::max();
    require(!config::validate_runtime_configuration(oversized), "overflowing M9025 speed conversion accepted");
    oversized = *example;
    oversized.m9025_motors[0].encoder_counts_per_turn = std::numeric_limits<float>::min();
    require(!config::validate_runtime_configuration(oversized), "sub-count M9025 encoder divisor accepted");
    example->m9025_motors[0].speed_rad_per_count = 0;
    require(!config::validate_runtime_configuration(*example), "unknown M9025 speed units accepted");

    const auto sentry_text = config::read_text_file(root / "configs/sentry.yaml");
    require(bool(sentry_text), "sentry source configuration missing");
    auto sentry = rfl::yaml::read<config::runtime_config, rfl::NoExtraFields, rfl::DefaultIfMissing>(*sentry_text);
    require(bool(sentry), "sentry schema must parse despite intentional topology rejection");
    require(!config::validate_runtime_configuration(*sentry), "source J6006/DJI 0x201 conflict accepted");
    // Test-only synthetic bus proves full staged topology validates once ownership is distinct.
    // It is not written into the hardware profile and asserts no installed wiring.
    sentry->cans.push_back({.name="test_isolated_bus", .interface_name="test_only"});
    sentry->j6006_motors[0].can_name = "test_isolated_bus";
    require(bool(config::validate_runtime_configuration(*sentry)), "isolated sentry test topology invalid");
    auto another = sentry->j6006_motors[0];
    another.id = 2; another.name = "test_second_j6006";
    sentry->j6006_motors.push_back(another); // Shared master, distinct controller IDs are valid.
    require(bool(config::validate_runtime_configuration(*sentry)), "shared J6006 master rejected");
    sentry->j6006_motors.back().id = 1;
    require(!config::validate_runtime_configuration(*sentry), "duplicate J6006 controller accepted");

    auto missing_id = rfl::json::write(good);
    const auto id_position = missing_id.find("\"id\":1,");
    require(id_position != std::string::npos, "missing-ID fixture not found");
    missing_id.erase(id_position, std::string{"\"id\":1,"}.size());
    auto parsed_missing = rfl::json::read<config::runtime_config, rfl::NoExtraFields, rfl::DefaultIfMissing>(missing_id);
    require(!parsed_missing || !config::validate_runtime_configuration(*parsed_missing), "missing motor ID accepted");

    const auto filename = std::filesystem::temp_directory_path() / "gkd-runtime-roundtrip.json";
    { std::ofstream file(filename); file << rfl::json::write(good); }
    require(bool(config::load_configuration(filename)), "owned info types JSON roundtrip failed");
    { std::ofstream file(filename); file << "{\"schema_version\":1,\"unexpected_field\":true}"; }
    require(!config::load_configuration(filename), "unknown configuration field accepted");
    std::filesystem::remove(filename);
}
