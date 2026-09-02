#include "device/chassis/base.hpp"

#include <mutex>
#include <unordered_map>
#include <utility>

namespace roboctrl::device {
namespace {

std::unordered_map<std::string, chassis_registry::factory>& chassis_factories() {
    static std::unordered_map<std::string, chassis_registry::factory> value;
    return value;
}

std::mutex& chassis_factories_mutex() {
    static std::mutex mutex;
    return mutex;
}

chassis_base*& current_chassis() {
    static chassis_base* value = nullptr;
    return value;
}

} // namespace

bool chassis_registry::register_type(std::string type, factory creator) {
    std::lock_guard lock{chassis_factories_mutex()};
    return chassis_factories().emplace(std::move(type), std::move(creator)).second;
}

chassis_base* chassis_registry::current() {
    return current_chassis();
}

chassis_base* chassis_registry::create(std::string_view type, const std::any& info) {
    factory creator;
    {
        std::lock_guard lock{chassis_factories_mutex()};
        const auto it = chassis_factories().find(std::string{type});
        if (it == chassis_factories().end()) return nullptr;
        creator = it->second;
    }
    return creator(info);
}

bool chassis_registry::init(std::string_view type, const std::any& info) {
    auto* result = create(type, info);
    current_chassis() = result;
    return result != nullptr;
}

} // namespace roboctrl::device
