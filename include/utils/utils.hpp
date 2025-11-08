#pragma once

#include "utils/concepts.hpp"
#include <concepts>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <cassert>
#include <numbers>
#include <type_traits>

namespace roboctrl{

using fp32 = float;
using fp64 = double;

template<typename T>
constexpr T Pi = std::numbers::pi_v<T>;
constexpr auto Pi_f = Pi<fp32>;


template<typename T>
    requires std::is_arithmetic_v<T>
struct vector{
    T x;
    T y;

    T norm() const{
        return std::sqrt(x * x + y * y);
    }

    vector normalized()const{
        return *this / norm();
    }
};

using vectori = vector<int>;
using vectorf = vector<float>;

template<typename T>
vector<T> operator + (vector<T> a,vector<T> b){
    return {a.x+b.x,a.y+b.y};
    
}

template<typename T>
vector<T> operator - (vector<T> a,vector<T> b){
    return {a.x-b.x,a.y-b.y};
}

template<typename T>
vector<T> operator * (vector<T> a,T b){
    return {a.x * b,a.y * b};
}

template<typename T>
vector<T> operator / (vector<T> a,T b){
    return {a.x / b,a.y / b};
}

namespace utils{


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

inline constexpr uint16_t make_u16(uint8_t high, uint8_t low) noexcept {
    return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
}

inline constexpr uint16_t make_i16(int8_t high, int8_t low) noexcept {
    return (static_cast<int16_t>(high) << 8) | static_cast<uint16_t>(low);
}

inline constexpr std::byte to_byte(std::integral auto v) noexcept{
    return static_cast<std::byte>(v);
}

}
}