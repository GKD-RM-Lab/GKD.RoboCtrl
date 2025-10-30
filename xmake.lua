add_rules("mode.debug","mode.release")

set_toolchains("llvm")
add_cxflags("--stdlib=libc++")
add_links("c++","c++abi")
add_syslinks("pthread")
add_requires("asio")
set_languages("c++23")

target("gkd-roboctrl")
    set_kind("binary")
    add_files("src/**.cpp")
    add_includedirs("include")
    add_packages("asio")

    if(is_mode("debug")) then 
        add_defines("GKD_DEBUG")
    end 
