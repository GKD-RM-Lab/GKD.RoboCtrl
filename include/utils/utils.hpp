/**
 * @file utils.hpp
 * @brief 常用数值与字节工具集合。
 * @details 定义浮点别名、二维向量操作、字节序列互转等通用帮助函数，方便在设备与通信模块间复用。
 */
#pragma once

#include <concepts>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <cassert>
#include <cmath>
#include <numbers>
#include <type_traits>

#include "utils/concepts.hpp"

namespace roboctrl{

/** @brief 单精度浮点别名。 */
using fp32 = float;
/** @brief 双精度浮点别名。 */
using fp64 = double;

/**
 * @brief 常量 Pi 的模板实例。
 * @tparam T 目标浮点类型
 */
template<typename T>
constexpr T Pi = std::numbers::pi_v<T>;

/** @brief 单精度浮点类型的 Pi 常量。 */
constexpr auto Pi_f = Pi<fp32>;


/**
 * @brief 简单的二维向量类型。
 * @tparam T 元素类型，要求为算术类型
 */
template<typename T>
    requires std::is_arithmetic_v<T>
struct vector{
    T x;
    T y;

    /**
     * @brief 获取向量欧氏范数。
     */
    T norm() const{
        return std::sqrt(x * x + y * y);
    }

    /**
     * @brief 获取该向量的单位化结果。
     */
    vector normalized()const{
        return *this / norm();
    }
};

using vectori = vector<int>;
using vectorf = vector<float>;

/** @internal */
template<typename T>
vector<T> operator + (vector<T> a,vector<T> b){
    return {a.x+b.x,a.y+b.y};
    
}

/** @internal */
template<typename T>
vector<T> operator - (vector<T> a,vector<T> b){
    return {a.x-b.x,a.y-b.y};
}

/** @internal */
template<typename T>
vector<T> operator * (vector<T> a,T b){
    return {a.x * b,a.y * b};
}

/** @internal */
template<typename T>
vector<T> operator / (vector<T> a,T b){
    return {a.x / b,a.y / b};
}

/**
 * @brief 用于存放工具函数的命名空间。
 * @details 封装了工具函数，如字节序列转换、角度处理等。
 */
namespace utils{

/**
* @brief 将角度限制到[-Pi,Pi]范围内
*/
constexpr inline fp32 rad_format(fp32 ang){
    fp32 ans = fmodf(ang + Pi_f, Pi_f * 2.f);
    return (ans < 0.f) ? ans + Pi_f : ans - Pi_f;
}

/**
 * @brief 描述具有 `data()` 与 `size()` 接口的字节容器。
 */
template<typename C>
concept byte_container =
    requires(C c) {
        { c.data() } -> std::convertible_to<const std::byte*>;
        { c.size() } -> std::convertible_to<std::size_t>;
    };

/**
 * @brief 将字节容器内容反序列化为平凡类型。
 * @tparam T 目标类型，需满足 `package`
 * @tparam C 字节容器类型
 * @param bytes 输入字节缓冲
 */
template<package T, byte_container C>
T from_bytes(const C& bytes) {
    assert(bytes.size() == sizeof(T));
    T t;
    std::memcpy(&t, bytes.data(), sizeof(T));
    return t;
}

/**
 * @brief 将平凡类型序列化到指定容器。
 * @tparam T 输入类型
 * @tparam C 可写容器类型
 * @param t 待写入对象
 * @param container 目标容器
 */
template<package T, typename C>
requires requires(C& c) {
    { c.data() } -> std::convertible_to<std::byte*>;
    { c.size() } -> std::convertible_to<std::size_t>;
}
void to_bytes(const T& t, C& container) {
    assert(container.size() >= sizeof(T));
    std::memcpy(container.data(), &t, sizeof(T));
}
/**
 * @brief 记录程序启动后的纳秒级时间戳。
 */
inline auto now() {
    static const auto init_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - init_time);
}

/**
 * @brief 将高低字节组合成无符号 16 位整数。
 */
inline constexpr uint16_t make_u16(uint16_t high, uint16_t low) noexcept {
    return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
}

/**
 * @brief 将高低字节组合成有符号 16 位整数。
 */
inline constexpr int16_t make_i16(int16_t high, int16_t low) noexcept {
    return (static_cast<int16_t>(high) << 8) | static_cast<int16_t>(low);
}

/**
 * @brief 辅助将整数转换为 `std::byte`。
 */
inline constexpr std::byte to_byte(std::integral auto v) noexcept{
    return static_cast<std::byte>(v);
}

template<typename T>
struct function_arg;

template<typename Ret, typename Arg>
struct function_arg<Ret(*)(Arg)> { using type = Arg; };

template<typename Ret, typename Class, typename Arg>
struct function_arg<Ret(Class::*)(Arg) const> { using type = Arg; };

template<typename Fn>
using function_arg_t = typename function_arg<decltype(&Fn::operator())>::type;

}
}
