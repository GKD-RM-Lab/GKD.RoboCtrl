#pragma once
#include <concepts>
#include <type_traits>
#include <utility>

#include "utils/concepts.hpp"

namespace roboctrl::utils{

/**
 * @brief 提供单例模式的单例基类
 * 
 * @tparam T 需要成为单例模式的类
 * @details 这是一个用于提供单例模式功能的基类。要编写一个单例类 T，你需要让 T 继承自 singleton_base<T>,
 * 并和多例模式一样添加一个用于初始化的 info_type 类型，并实现一个名为 init() 的成员函数，该函数接受 info_type 类型的参数。
 * 在初始化时，init() 函数会被调用，你应该通过其返回值是 true 还是 false 来表明是否成功初始化。
 */
template <typename T>
class singleton_base : public utils::immovable_base, public utils::not_copyable_base {
public:
    singleton_base(const singleton_base&) = delete;
    singleton_base& operator=(const singleton_base&) = delete;

    /**
     * @brief 获取单例实例
     * 
     * @return T& 单例实例
     */
    static T& instance() {
        static T instance;
        return instance;
    }

protected:
    singleton_base() = default;
    virtual ~singleton_base() = default;
};

/**
 * @brief 单例信息
 * @details 和roboctrl::multiton_info类似，但不需要key_type和key函数
 * 
 * @tparam owner_type 单例的拥有者类型
 */
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