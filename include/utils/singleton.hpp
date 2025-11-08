#pragma once
#include "utils/concepts.hpp"
#include <concepts>
#include <type_traits>
#include <utility>

namespace roboctrl::utils{

template <typename T>
class singleton_base : public utils::immovable_base, public utils::not_copyable_base {
public:
    singleton_base(const singleton_base&) = delete;
    singleton_base& operator=(const singleton_base&) = delete;
    static T& instance() {
        static T instance;
        return instance;
    }

protected:
    singleton_base() = default;
    virtual ~singleton_base() = default;
};

template<typename T>
concept singleton_info = requires (T info) {
    typename T::owner_type;
};

template<typename T>
concept singleton = std::is_base_of_v<singleton_base<T>, T> and
    requires (T t) {
        typename T::info_type;
        {t.init(std::declval<typename T::info_type>())} -> std::same_as<bool>;
    };
}