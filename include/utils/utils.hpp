#pragma once

#include "utils/concepts.hpp"
#include <cstring>
#include <chrono>

namespace roboctrl::utils{
template<typename C>
concept byte_container =
    requires(C c) {
        { c.data() } -> std::convertible_to<const std::byte*>;
        { c.size() } -> std::convertible_to<std::size_t>;
    };

template<package T, byte_container C>
T from_bytes(const C& bytes) {
    assert(bytes.size() == sizeof(T));
    T t;
    std::memcpy(&t, bytes.data(), sizeof(T));
    return t;
}

template<package T, typename C>
requires requires(C& c) {
    { c.data() } -> std::convertible_to<std::byte*>;
    { c.size() } -> std::convertible_to<std::size_t>;
}
void to_bytes(const T& t, C& container) {
    assert(container.size() >= sizeof(T));
    std::memcpy(container.data(), &t, sizeof(T));
}

inline auto now() {
    static const auto init_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - init_time);
}

}