add_rules("mode.debug", "mode.release")

set_toolchains("llvm")
set_languages("c++23")
add_cxxflags("-stdlib=libc++","-fexceptions","-frtti")
add_ldflags("-stdlib=libc++", "-lc++", "-lc++abi")
add_syslinks("pthread")
add_requires("asio", "cxxopts")

option("type")
    set_default("infantry")
    set_showmenu(true)
    set_description("指定要编译的机器人类型")
    set_values("infantry","hero","sentry","project")
    after_check(function(option)
        local val = option:value()
        local type = 0

        if val == "infantry" then 
            type = 1
        elseif val == "hero" then 
            type = 2
        elseif val == "sentry" then 
            type = 3
        elseif val == "project" then 
            type = 4
        else
            raise("Invaild type : " .. val)
        end 

        option:add("defines", "BUILD_TYPE=" .. type)
    end)

target("gkd-roboctrl")
    set_kind("binary")
    add_files("src/**.cpp")
    add_includedirs("include")
    add_packages("asio", "cxxopts")
    add_options("type")

    if get_config("type") then
        set_basename("gkd.roboctrl." .. get_config("type"))
    end

    if is_mode("debug") then
        add_defines("DEBUG")
    end
