/**
 * @file base.hpp
 * @brief 运行时车型选择的兼容入口。
 * @details 车型不再通过编译宏选择；程序启动后从 YAML/JSON 配置文件读取
 *          profile 和完整硬件拓扑。该头文件仅保留默认配置名，避免旧 include
 *          点在移除编译期车型选择后失效。
 */
#pragma once

#include <string_view>

namespace roboctrl::config {

/** @brief 未指定 `--config` 时使用的默认配置文件名。 */
inline constexpr std::string_view default_profile = "infantry";

} // namespace roboctrl::config
