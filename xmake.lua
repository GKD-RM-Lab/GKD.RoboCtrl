add_rules("mode.debug", "mode.release")

set_toolchains("llvm")
set_languages("c++23")
add_cxxflags("-stdlib=libc++","-fexceptions","-frtti", {tools = "clang"})
add_ldflags("-stdlib=libc++", "-lc++", "-lc++abi")
add_syslinks("pthread")
add_requires("asio", "cxxopts","iguana")

target("gkd-roboctrl")
    set_kind("binary")
    add_files("src/**.cpp")
    add_includedirs("include")
    add_packages("asio", "cxxopts","iguana")

    add_defines("ASIO_STANDALONE", "ASIO_HAS_STD_COROUTINE=1", "ASIO_HAS_CO_AWAIT=1","ASIO_HAS_STD_EXCEPTION_PTR")

    if is_mode("debug") then
        add_defines("DEBUG")
    end
