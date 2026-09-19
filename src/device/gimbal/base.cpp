#include "device/gimbal/base.hpp"

#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace roboctrl::device {
namespace {

std::unordered_map<std::string, gimbal_registry::factory>& gimbal_factories() {
    static std::unordered_map<std::string, gimbal_registry::factory> value;
    return value;
}

std::mutex& gimbal_factories_mutex() {
    static std::mutex mutex;
    return mutex;
}

gimbal_base*& current_gimbal() {
    static gimbal_base* value = nullptr;
    return value;
}

} // namespace

bool gimbal_registry::register_type(std::string type, factory creator) {
    std::lock_guard lock{gimbal_factories_mutex()};
    return gimbal_factories().emplace(std::move(type), std::move(creator)).second;
}

gimbal_base* gimbal_registry::current() {
    return current_gimbal();
}

gimbal_base* gimbal_registry::create(std::string_view type, const std::any& info) {
    factory creator;
    {
        std::lock_guard lock{gimbal_factories_mutex()};
        const auto it = gimbal_factories().find(std::string{type});
        if (it == gimbal_factories().end()) return nullptr;
        creator = it->second;
    }
    auto result = creator(info);
    if (!result) return nullptr;
    auto* pointer = result.get();
    // Controllers and their coroutines live for the process lifetime.
    static std::vector<std::unique_ptr<gimbal_base>> instances;
    instances.push_back(std::move(result));
    return pointer;
}

bool gimbal_registry::init(std::string_view type, const std::any& info) {
    auto* result = create(type, info);
    current_gimbal() = result;
    return result != nullptr;
}

} // namespace roboctrl::device
