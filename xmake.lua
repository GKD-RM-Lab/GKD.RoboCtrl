add_rules("mode.debug", "mode.release")

set_toolchains("llvm")
set_languages("c++23")
add_cxxflags("-stdlib=libc++","-fexceptions","-frtti")
add_ldflags("-stdlib=libc++", "-lc++", "-lc++abi")
add_syslinks("pthread")
add_requires("asio", "cxxopts")
add_requires("reflect-cpp v0.25.0", {configs = {yaml = true}})

target("gkd-roboctrl")
    set_kind("binary")
    add_files("src/**.cpp")
    add_includedirs("include")
    add_packages("asio", "cxxopts", "reflect-cpp")

    if is_mode("debug") then
        add_defines("DEBUG")
    end

target("unit-tests")
    set_default(false)
    set_kind("binary")
    add_files("tests/unit_tests.cpp", "tests/configuration_migration_tests.cpp",
              "tests/motor_protocol_tests.cpp", "tests/network_protocol_tests.cpp",
              "tests/referee_protocol_tests.cpp", "tests/shoot_integration_tests.cpp",
              "tests/motion_migration_tests.cpp", "tests/power_control_tests.cpp",
              "tests/ballistics_tests.cpp", "src/device/base.cpp",
              "src/device/gimbal/base.cpp", "src/device/gimbal/gkd_sentry_gimbal.cpp",
              "src/device/referee/referee.cpp", "src/ctrl/shoot.cpp",
              "src/ctrl/power_manager.cpp", "src/utils/ballistics.cpp",
              "src/io/serial.cpp", "src/core/async.cpp", "src/core/logger.cpp")
    add_cxxflags("-UNDEBUG")
    add_includedirs("include")
    add_packages("asio", "reflect-cpp")

target("config-check")
    set_default(false)
    set_kind("binary")
    add_files("tools/config_check.cpp", "src/ctrl/power_manager.cpp")
    add_includedirs("include")
    add_packages("asio", "reflect-cpp")

target("ballistics-cli")
    set_default(false)
    set_kind("binary")
    add_files("tools/ballistics_cli.cpp", "src/utils/ballistics.cpp")
    add_includedirs("include")

target("network-transport-tests")
    set_default(false)
    set_kind("binary")
    add_files("tests/network_transport_tests.cpp", "src/io/udp_server.cpp",
              "src/device/aim_link.cpp", "src/device/remote_logger.cpp",
              "src/device/base.cpp", "src/core/async.cpp", "src/core/logger.cpp")
    add_includedirs("include")
    add_packages("asio")
