/**
 * @file runtime.hpp
 * @brief 外部 JSON/YAML 配置的读取与启动前校验。
 *
 * 配置根对象直接组合各组件原有的 info_type；不维护平行的 Spec 类型。
 */
#pragma once

#include <algorithm>
#include <expected>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <ostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <rfl/thirdparty/enchantum/enchantum.hpp>

#include "config/base.hpp"
#include "config/validate.hpp"
#include "ctrl/power_manager.h"
#include "device/aim_link.hpp"
#include "device/remote_logger.hpp"
#include "device/referee/referee.h"
#include "device/super_cap.h"
#include "device/motor/j6006.h"
#include "device/motor/m9025.h"

// DJI 型号枚举使用了实际型号作为数值（2006/3508/6020），超出
// reflect-cpp 的默认枚举扫描范围。仅在配置层为该既有枚举设置扫描范围，
// 不改变设备层，也不引入平行的配置类型。
namespace enchantum {

template<>
struct enum_traits<roboctrl::device::dji_motor::type> {
    static constexpr std::size_t prefix_length = 0;
    static constexpr int min = 2006;
    static constexpr int max = 6020;
};

} // namespace enchantum

#include <rfl/NoExtraFields.hpp>
#include <rfl/Processors.hpp>
#include <rfl/json.hpp>
#include <rfl/yaml.hpp>

namespace roboctrl::config {

/**
 * @brief 运行时配置根对象。
 * @details 根对象直接组合各设备的 info_type；vector 允许由文件决定实例数量。
 */
struct runtime_config {
    unsigned int schema_version {1};
    std::string profile;

    std::vector<io::can::info_type> cans;
    std::vector<io::serial::info_type> serials;
    std::vector<device::dji_motor::info_type> dji_motors;
    std::vector<device::j6006::info_type> j6006_motors;
    std::vector<device::m9025::info_type> m9025_motors;
    std::vector<device::serial_imu::info_type> additional_imus;
    std::vector<io::udp_server::info_type> udp_servers;
    std::vector<device::aim_link::info_type> aim_links;
    std::vector<device::navigation_link::info_type> navigation_links;
    std::vector<device::remote_logger::info_type> remote_loggers;
    std::optional<device::referee::info_type> referee;
    std::optional<device::super_cap::info_type> super_cap;
    std::optional<ctrl::power_manager::info_type> power;

    device::control_pad::info_type control_pad;
    device::serial_imu::info_type imu;
    ctrl::robot::info_type robot;
};

using configuration_result = std::expected<runtime_config, std::string>;

/** Check optional application links and policies before opening any IO resource. */
inline void validate_runtime_extensions(const runtime_config& config) {
    const auto require = [](bool value, const std::string& message) {
        if (!value) throw std::invalid_argument(message);
    };
    const auto keys = []<typename Info>(const std::vector<Info>& items, std::string_view kind) {
        validate_unique_keys(kind, std::span<const Info>{items});
        std::unordered_set<std::string> result;
        for (const auto& item : items) result.insert(item.key());
        return result;
    };
    const auto servers = keys(config.udp_servers, "UDP server");
    const auto aims = keys(config.aim_links, "aim link");
    const auto navigation = keys(config.navigation_links, "navigation link");
    const auto loggers = keys(config.remote_loggers, "remote logger");
    std::unordered_set<std::string> listen_endpoints, protocol_routes;
    const auto endpoint = [&](const auto& info) {
        asio::error_code error;
        const auto address = asio::ip::make_address(info.address, error);
        require(!error && address.is_v4() && info.port > 0,
                "network endpoints require a numeric IPv4 address and a nonzero port");
    };
    for (const auto& server : config.udp_servers) {
        endpoint(server);
        require(listen_endpoints.insert(std::to_string(server.port)).second,
                "duplicate UDP listen port (including wildcard address collisions)");
    }
    const auto link = [&](const auto& item, unsigned header) {
        endpoint(item);
        require(servers.contains(item.udp_name), "network link references missing UDP server");
        require(protocol_routes.insert(std::format("{}:{}", item.udp_name, header)).second,
                "duplicate application header on shared UDP socket");
    };
    for (const auto& item : config.aim_links) {
        require(item.header != 0x37, "vision header 0x37 is reserved for navigation");
        link(item, item.header);
        require(item.target_timeout_ms > 0 && item.target_timeout_ms <= 10000,
                "aim target timeout must be in 1..10000 ms");
        require(item.wire_format == device::vision_wire_format::legacy_14 ||
                item.wire_format == device::vision_wire_format::compact_10,
                "unknown vision wire format");
    }
    for (const auto& item : config.navigation_links) {
        link(item, 0x37);
        require(item.target_timeout_ms > 0 && item.target_timeout_ms <= 10000,
                "navigation timeout must be in 1..10000 ms");
    }
    for (const auto& item : config.remote_loggers) {
        endpoint(item);
        require(servers.contains(item.udp_name), "remote logger references missing UDP server");
    }
    const auto& motion = config.robot.motion_info;
    require(motion.primary_aim_key.empty() || (config.robot.enable_gimbal && aims.contains(motion.primary_aim_key)),
            "primary aim references missing link or disabled gimbal");
    require(motion.secondary_aim_key.empty() || (config.robot.secondary_gimbal_info && aims.contains(motion.secondary_aim_key)),
            "secondary aim requires a secondary gimbal and existing link");
    require(!motion.enable_navigation || (config.robot.enable_chassis && navigation.contains(motion.navigation_key)),
            "navigation control requires chassis and existing navigation link");
    require(motion.navigation_key.empty() || navigation.contains(motion.navigation_key), "missing navigation telemetry link");
    require(motion.remote_logger_key.empty() || loggers.contains(motion.remote_logger_key), "missing motion remote logger");
    require(motion.telemetry_period > std::chrono::steady_clock::duration::zero(), "invalid telemetry period");
    require(std::isfinite(motion.large_yaw_follow_direction) && std::fabs(motion.large_yaw_follow_direction) == 1,
            "large yaw follow direction must be +1 or -1");
    if (config.referee) {
        const auto& item = *config.referee;
        const auto serial = std::ranges::find(config.serials, item.serial_name, &io::serial::info_type::name);
        require(serial != config.serials.end() && serial->raw, "referee requires an existing raw serial");
        require(item.offline_timeout > std::chrono::steady_clock::duration::zero() &&
                item.ui_period >= std::chrono::milliseconds{100}, "invalid referee timeout/UI period");
    }
    if (config.robot.enable_shoot) {
        require(!config.robot.shoot_info.enforce_referee || config.referee.has_value(),
                "referee-gated shoot requires referee configuration");
        require(!config.robot.secondary_shoot_info || !config.robot.secondary_shoot_info->enforce_referee || config.referee.has_value(),
                "secondary referee-gated shoot requires referee configuration");
    }
    if (config.super_cap) {
        const auto& item = *config.super_cap;
        require(device::super_cap::valid_configuration(item), "invalid super capacitor configuration");
        require(std::ranges::find(config.cans, item.can_name, &io::can::info_type::name) != config.cans.end(),
                "super capacitor references missing CAN");
        const auto conflict = [&](unsigned id) {
            for (const auto& m : config.dji_motors) if (m.can_name == item.can_name) {
                const bool gimbal = m.type_ == device::dji_motor::M6020;
                if (id == (gimbal ? 0x204u : 0x200u) + m.id ||
                    id == (gimbal ? (m.id <= 4 ? 0x1ffu : 0x2ffu) : (m.id <= 4 ? 0x200u : 0x1ffu))) return true;
            }
            for (const auto& m : config.j6006_motors)
                if (m.can_name == item.can_name && (id == m.master_id || id == 0x200u + m.id)) return true;
            for (const auto& m : config.m9025_motors)
                if (m.can_name == item.can_name && id == 0x140u + m.id) return true;
            return false;
        };
        require(!conflict(item.receive_id) && !conflict(item.command_id), "super capacitor CAN ID conflicts with motor");
    }
    if (config.power) {
        require(ctrl::power_manager::valid_configuration(*config.power), "invalid power manager configuration");
        require(!config.power->enabled || config.robot.enable_chassis, "power manager requires enabled chassis");
    }
}


/**
 * @brief 校验已解析的运行时配置。
 * @param config 待校验的配置对象；成功时按值返回，便于继续移动到初始化阶段。
 * @return 通过校验的配置，或包含错误原因的 `std::unexpected`。
 */
inline configuration_result validate_runtime_configuration(runtime_config config) {
    if (config.schema_version != 1) {
        return std::unexpected("unsupported configuration schema_version: " +
                               std::to_string(config.schema_version));
    }
    if (config.robot.enable_chassis && config.robot.chassis_type.empty()) {
        return std::unexpected("chassis_type must not be empty when chassis is enabled");
    }
    if (config.robot.enable_gimbal && config.robot.gimbal_type.empty()) {
        return std::unexpected("gimbal_type must not be empty when gimbal is enabled");
    }
    try {
        validate_configuration(
            std::span{config.cans},
            std::span{config.serials},
            std::span{config.dji_motors},
            config.control_pad,
            config.imu,
            config.robot,
            std::span{config.additional_imus},
            std::span{config.j6006_motors},
            std::span{config.m9025_motors});
        validate_runtime_extensions(config);
    } catch (const std::exception& error) {
        return std::unexpected(error.what());
    }
    return config;
}

/**
 * @brief 读取整个文本文件。
 * @param path 文件路径。
 * @return 文件内容，或包含打开失败原因的 `std::unexpected`。
 */
inline std::expected<std::string, std::string> read_text_file(
    const std::filesystem::path& path)
{
    std::ifstream input{path};
    if (!input) {
        return std::unexpected("cannot open configuration file: " + path.string());
    }
    return std::string{
        std::istreambuf_iterator<char>{input},
        std::istreambuf_iterator<char>{}};
}

/**
 * @brief 从 YAML 或 JSON 文件加载并校验运行时配置。
 * @param path 配置文件路径；扩展名必须为 `.yaml`、`.yml` 或 `.json`。
 * @return 已解析且通过语义校验的配置，或包含解析/校验错误的 `std::unexpected`。
 */
inline configuration_result load_configuration(const std::filesystem::path& path) {
    const auto text = read_text_file(path);
    if (!text) return std::unexpected(text.error());

    const auto extension = path.extension().string();
    if (extension == ".yaml" || extension == ".yml") {
        auto parsed = rfl::yaml::read<runtime_config, rfl::NoExtraFields,
                                      rfl::DefaultIfMissing>(*text);
        if (!parsed) {
            return std::unexpected("failed to parse " + path.string() + ": " +
                                   parsed.error().what());
        }
        return validate_runtime_configuration(std::move(*parsed));
    }
    if (extension == ".json") {
        auto parsed = rfl::json::read<runtime_config, rfl::NoExtraFields,
                                      rfl::DefaultIfMissing>(*text);
        if (!parsed) {
            return std::unexpected("failed to parse " + path.string() + ": " +
                                   parsed.error().what());
        }
        return validate_runtime_configuration(std::move(*parsed));
    }
    return std::unexpected("unsupported configuration extension: " + extension);
}

/**
 * @brief 获取未指定 `--config` 时使用的默认配置路径。
 * @return `configs/infantry.yaml`（默认配置；其他车型通过 `--config` 选择）。
 */
inline std::filesystem::path default_configuration_path() {
    return std::filesystem::path{"configs"} /
           (std::string{default_profile} + ".yaml");
}

/**
 * @brief 将指定目录下的 YAML/JSON 配置按名称排序后打印到输出流。
 * @param output 输出流。
 * @param directory 要枚举的配置目录。
 */
inline void print_available_configurations(
    std::ostream& output,
    const std::filesystem::path& directory)
{
    output << "\n=== Configuration files in " << directory.string() << " ===\n";
    if (!std::filesystem::exists(directory)) {
        output << "(directory does not exist)\n";
        return;
    }

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator{directory}) {
        if (!entry.is_regular_file()) continue;
        const auto extension = entry.path().extension().string();
        if (extension == ".yaml" || extension == ".yml" || extension == ".json") {
            files.push_back(entry.path());
        }
    }
    std::ranges::sort(files);
    if (files.empty()) {
        output << "(no YAML or JSON configuration files)\n";
        return;
    }

    for (const auto& file : files) {
        output << "--- " << file.string() << " ---\n";
        const auto text = read_text_file(file);
        if (!text) {
            output << "<" << text.error() << ">\n";
            continue;
        }
        output << *text;
        if (!text->empty() && text->back() != '\n') output << '\n';
    }
}

} // namespace roboctrl::config
