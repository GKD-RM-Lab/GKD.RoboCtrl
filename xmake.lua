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
    add_files("tests/unit_tests.cpp", "src/device/base.cpp")
    add_includedirs("include")
    add_packages("asio", "reflect-cpp")
