#pragma once

#include <type_traits>
#include <utility>
#include <cstddef>
#include <stdexcept>

namespace roboctrl::utils{
    template <typename F, typename Ret, typename... Args>
concept invocable_r = std::is_invocable_r_v<Ret, F, Args...>;

namespace detail {
template <typename T, template <typename...> class S>
struct instance_of_impl : std::false_type {};
template <template <typename...> class S, typename... Args>
struct instance_of_impl<S<Args...>, S> : std::true_type {};
} // namespace detail
template <typename T, template <typename...> class S>
concept instance_of = detail::instance_of_impl<T, S>::value;

template <typename T>
concept immovable = 
    (not std::is_move_constructible_v<T>) and
    (not std::is_move_assignable_v<T>);
template <typename T>
concept not_copyable = 
    (not std::is_copy_constructible_v<T>) and
    (not std::is_copy_assignable_v<T>);

struct immovable_base {
    immovable_base() = default;
    immovable_base(immovable_base&&) = delete;
    immovable_base& operator=(immovable_base&&) = delete;
};
struct not_copyable_base {
    not_copyable_base() = default;    
    not_copyable_base(const not_copyable_base&) = delete;
    not_copyable_base& operator=(const not_copyable_base&) = delete;
};

template<typename T>
concept package = std::is_trivially_copyable_v<T>;

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

namespace byte_literals {
consteval std::byte operator""_b(unsigned long long int byte) {
    if (byte > 0xff) throw std::out_of_range{ "std::byte literal exceeds 0xff." };
    return static_cast<std::byte>(byte);
}
} // namespace byte_literals
}