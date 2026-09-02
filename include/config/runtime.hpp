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
#include <span>
#include <string>
#include <vector>

#include <rfl/thirdparty/enchantum/enchantum.hpp>

#include "config/base.hpp"
#include "config/validate.hpp"

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

    device::control_pad::info_type control_pad;
    device::serial_imu::info_type imu;
    ctrl::robot::info_type robot;
};

using configuration_result = std::expected<runtime_config, std::string>;

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
            config.robot);
    } catch (const std::exception& error) {
        return std::unexpected(error.what());
    }
    return config;
}

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

inline std::filesystem::path default_configuration_path() {
    return std::filesystem::path{"configs"} /
           (std::string{TYPE_STR} + ".yaml");
}

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
