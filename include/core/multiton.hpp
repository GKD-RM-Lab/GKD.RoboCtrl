/**
 * @file multiton.hpp
 * @author Junity
 * @brief 用于实现多例模式的通用组件。
 * @details 多例模式允许根据不同的key获取对应的实例，适用于需要管理多个相似对象的场景，例如马达、传感器等等。
 *
 * 实现一个多例类需要定义一个info_type结构体，包含key_type和owner_type类型别名，以及一个key()成员函数用于获取实例的唯一标识符。
 * 除此之外，还应该包含所有用于初始化这个多例类的信息。在初始化时，roboctrl::init()函数会将info_type传递给多例类的构造函数进行初始化。在程序
 * 开始时，可以通过调用multiton::init函数初始化所需的多例实例。之后，可以通过roboctrl<T>::get(key)根据key获取对应的实例。
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <concepts>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <format>
#include <utility>

#include "utils/concepts.hpp"
#include "utils/singleton.hpp"

namespace roboctrl{

/**
 * @brief 多例相关的基础组件
 * @details 在机器人的电控中，有很多类是会在初始化时创建多个对象，并且生命周期几乎是从程序开始到结束，例如马达对象，Can对象等等
 * ，我们称这种对象为多例。我们希望有一个简单的拿到各个实例的方法。
 *
 * 在代码中，我们抽象出了一个多例类的管理模式：每个多例类有一个info_type，用于存放用来初始化这个类的相关信息。info_type中有一个
 * 叫做 key_type 的类型，这是用于区分各个多例对象的“标识符”。例如，can对象的key_type是一个字符串，表示这个can的名称；那么如果有一
 * 个can叫"can0"，在初始化后，就可以用 roboctrl::get<can_io>("can0") 来获取到这个can对象的引用。
 *
 * 为了满足这个管理模式，所有的多例类都应该有一个接受 const info_type& 参数的构造函数，用于初始化这个多例对象。这个函数会在程序开
 * 始时，在roboctrl::init()函数中被调用。
 */
namespace multiton{
/**
 * @brief 拥有描述信息的类。
 * @details 这些类有一个desc()函数用于获取人类可读的描述信息，通常用于日志输出。
 */
template <typename T>
concept descable = requires (const T& t) {
    { t.desc() } -> std::same_as<std::string>;
};

/**
 * @brief 多例类的info_type concept
 * @details info_type需要定义key_type和owner_type类型别名，以及key()成员函数。其中，key_type表示实例的唯一标识符类型
 * ，owner_type表示多例类本身的类型。
 */
template<typename T>
concept multiton_info = requires(T info){
    typename T::key_type;
    typename T::owner_type;
    {info.key()} -> std::same_as<typename T::key_type>;
};

/**
 * @brief 通用的info concept，主要用于统一单例和多例的info概念。这个concept的主要目的是让单例和多例都可以被roboctrl::get获取。
 */
template<typename T>
concept info = multiton_info<T> or utils::singleton_info<T>;

/**
 * @brief 描述多例类的concept
 * @details 一个多例类应该包含应该info_type，用于初始化多例实例，以及获取实例的key，并且有一个接受const info_type&的构造函数
 * ；此外，还需要实现desc()成员函数用于描述实例信息。
 *
 * 在初始化阶段，应该使用roboctrl::init()函数传入info_type对象来创建并初始化多例实例。由于多例对象间有依赖关系（例如马达依赖CAN对象），
 * 因此需要注意初始化顺序，确保依赖的多例对象先被初始化。
 */
template<typename T>
concept owner = requires(T owner){
    typename T::info_type;
    {T(std::declval<typename T::info_type>())};
};

/// @internal
template<multiton_info T>
using key_type_t = typename T::key_type;


/// @internal
template<multiton_info T>
using owner_type_t = typename T::owner_type;

/// @internal
namespace details{

template <typename owner_type>
struct multiton_impl final :
    public utils::immovable_base,
    public utils::not_copyable_base {

    using key_type = typename owner_type::info_type::key_type;
    
    static std::mutex mutex_;
    static std::unordered_map<key_type, std::unique_ptr<owner_type>> instances;
    
    using info_type = typename owner_type::info_type;

    static void init(const info_type& info){
        std::lock_guard<std::mutex> lock{mutex_};

        auto instance = std::make_unique<owner_type>(info);
        owner_type& ret = *instance;
        instances.emplace(info.key(), std::move(instance));
    }

    [[nodiscard]]
    static bool contains(const key_type& key){
        return instances.contains(key);
    }

    [[nodiscard]]
    static auto get(const key_type& key) -> owner_type& {
        std::lock_guard<std::mutex> lock { mutex_ };
        auto it = instances.find(key);
        if (it != instances.end()) {
            return *it->second;
        }
        
        throw std::runtime_error(std::format("uninitialized multiton {}",key)); //TODO:add detailed desc.
    };
};

template<multiton_info info_type>
using impl_t = multiton_impl<owner_type_t<info_type>>;

template <typename owner_type>
std::mutex multiton_impl<owner_type>::mutex_ {};

template <typename owner_type>
std::unordered_map<
    typename multiton_impl<owner_type>::key_type, 
    std::unique_ptr<owner_type>
> multiton_impl<owner_type>::instances {};
}

/**
 * @brief 获取多例实例
 * @details 通过指定的key来获取多例对象
 *
 * 示例：获取指定的CAN对象：
 * ```cpp
 * auto& can1 = roboctrl::get<roboctrl::io::can>("can1");
 * ```
 * 
 * @tparam owner_type 多例类类型
 * @param key 用于获取实例的key
 * @return owner_type& 多例对象的引用
 */
template<owner owner_type>
[[nodiscard]]
inline auto get(const typename details::multiton_impl<owner_type>::key_type& key) -> owner_type&{
    return details::multiton_impl<owner_type>::get(key);
}

/**
 * @brief 获取单例实例
 * 
 * @tparam T 单例类类型
 * @return T& 单例对象的引用
 * @details 根据类型获取单例对象
 *
 * 示例：获取Gimbal控制器单例对象：
 * ```cpp
 * auto& gimbal_ctrl = roboctrl::get<roboctrl::ctrl::gimbal>();
 * ```
 */
template<typename T>
    requires(!owner<T> && utils::singleton<T>)
inline auto get() -> T&{
    return T::instance();
}

/**
 * @brief 初始化多例实例或单例实例
 * 
 * @tparam info_type 多例类或单例类的info_type类型
 * @param info 多例类或单例类的info_type对象
 * @details 用给定的info_type初始化多例实例或单例实例
 *
 * 示例：初始化CAN对象：
 * ```cpp
 * roboctrl::init(
 *     roboctrl::io::can::info_type{
 *         "can1"
 *     }
 * );
 * ```
 */

template<info info_type>
inline auto init(const info_type& info) -> bool{
    if constexpr (multiton_info<info_type>){
        details::impl_t<info_type>::init(info);
        return true;
    }
    else return get<typename info_type::owner_type>().init(info);
}

template<info info_type>
inline auto init(std::initializer_list<info_type> infos) -> bool{
    for(auto info:infos)
        if(!init(info))
            return false;
    
    return true;
}

/**
 * @brief 批量初始化多例实例或单例实例
 * 
 * @tparam info_types 多例类或单例类的info_type类型
 * @param infos 多例类或单例类的info_type对象
 * @details 用给定的info_type批量初始化多例实例或单例实例，可以在这个函数中初始化不同类型的多例或单例对象。
 */
template<info... info_types>
inline auto init(const info auto& info,const info_types&... infos) -> bool{
    if(init(info))
        return init(infos...);
    return false;
}

/**
 * @brief 通过info_type获取多例实例
 * 
 * @tparam info_type 多例类的info_type类型
 * @param info 多例类的info_type对象
 * @return owner_type_t<info_type>& 多例对象的引用
 */
template<multiton_info info_type>
[[nodiscard]]
inline auto get(const info_type& info) -> owner_type_t<info_type>&{
    using owner_type = owner_type_t<info_type>;
    auto key = info.key();
    if(!details::multiton_impl<owner_type>::contains(key))
        details::multiton_impl<owner_type>::init(info);

    return details::multiton_impl<owner_type>::get(key);
}

/**
 * @brief 获取一个可描述对象的描述信息
 * 
 * @tparam T 多例类类型
 * @param owner 多例类对象
 * @return std::string 多例类的描述信息
 */
[[nodiscard]]
inline auto desc(const descable auto& owner) -> std::string{
    return owner.desc();
}

}

using namespace multiton;
}