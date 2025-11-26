#pragma once

#include <functional>
#include <utility>

#define DEFER_CAT_IMPL(x, y) x##y
#define DEFER_CAT(x, y) DEFER_CAT_IMPL(x, y)

#define defer(...) auto DEFER_CAT(_defer_, __COUNTER__) = ::roboctrl::utils::detail::Defer([&](){ __VA_ARGS__; })

namespace roboctrl::utils{
namespace detail {
    struct Defer {
        template<class F>
        Defer(F&& f) : fn(std::forward<F>(f)) {}

        ~Defer() { fn(); }

        std::function<void()> fn;
    };
}
}