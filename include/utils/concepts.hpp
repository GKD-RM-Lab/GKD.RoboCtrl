/**
 * @file concepts.hpp
 * @brief 公共Concept与元类型工具。
 * @details 提供一组可在全项目复用的Concept、类型萃取工具以及字节面向的辅助结构，方便统一约束模板接口。
 */
#pragma once

#include <type_traits>
#include <utility>
#include <cstddef>
#include <stdexcept>

namespace roboctrl::utils{

/**
 * @brief 约束可调用对象的返回值。
 * @tparam F 待检查的可调用类型
 * @tparam Ret 期望的返回类型
 * @tparam Args 调用参数类型列表
 */
template <typename F, typename Ret, typename... Args>
concept invocable_r = std::is_invocable_r_v<Ret, F, Args...>;

namespace detail {
template <typename T, template <typename...> class S>
struct instance_of_impl : std::false_type {};
template <template <typename...> class S, typename... Args>
struct instance_of_impl<S<Args...>, S> : std::true_type {};
} // namespace detail
/**
 * @brief 判断类型是否为某模板的实例。
 * @tparam T 需要判断的类型
 * @tparam S 模板模板参数
 */
template <typename T, template <typename...> class S>
concept instance_of = detail::instance_of_impl<T, S>::value;

/**
 * @brief 判断类型不可移动。
 * @tparam T 类型参数
 */
template <typename T>
concept immovable = 
    (not std::is_move_constructible_v<T>) and
    (not std::is_move_assignable_v<T>);

/**
 * @brief 判断类型不可复制。
 * @tparam T 类型参数
 */
template <typename T>
concept not_copyable = 
    (not std::is_copy_constructible_v<T>) and
    (not std::is_copy_assignable_v<T>);

/**
 * @brief 快速继承获得“不可移动”约束的基类。
 */
struct immovable_base {
    immovable_base() = default;
    immovable_base(immovable_base&&) = delete;
    immovable_base& operator=(immovable_base&&) = delete;
};

/**
 * @brief 快速继承获得“不可复制”约束的基类。
 */
struct not_copyable_base {
    not_copyable_base() = default;    
    not_copyable_base(const not_copyable_base&) = delete;
    not_copyable_base& operator=(const not_copyable_base&) = delete;
};

/**
 * @brief 用于约束“可安全作为报文搬运”的平凡类型。
 */
template<typename T>
concept package = std::is_trivially_copyable_v<T>;

/**
 * @brief 双值绑定结构，兼容 std::pair。
 * @details 提供 first/second 及 left/right 两套更语义化的字段别名，便于表达数据语义。
 */
template <typename T, typename U = T>
struct pair {
    std::pair<T, U> p;
    T& first { p.first };
    T& left { p.first };
    U& second { p.second };
    U& right { p.second };
    explicit pair(auto&&... args) : p { std::forward<decltype(args)>(args)... } {}
    pair() = default;
    operator std::pair<T, U>& () { return p; }
};

/**
 * @brief 字节面向的用户字面量命名空间。
 */
namespace byte_literals {
/**
 * @brief 将无符号整型面量转换为 `std::byte`。
 * @param byte 字面量数值，范围需小于等于 0xFF
 * @throws std::out_of_range 数值超出 1 字节取值范围时抛出
 */
consteval std::byte operator""_b(unsigned long long int byte) {
    if (byte > 0xff) throw std::out_of_range{ "std::byte literal exceeds 0xff." };
    return static_cast<std::byte>(byte);
}
} // namespace byte_literals
}
