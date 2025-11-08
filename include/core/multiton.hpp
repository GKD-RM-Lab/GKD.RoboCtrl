#pragma once

#include <concepts>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <utility>

#include "utils/concepts.hpp"
#include "utils/singleton.hpp"
#include "core/logger.h"

namespace roboctrl{

template<typename T>
concept multiton_info = requires(T info){
    typename T::key_type;
    typename T::owner_type;
    {info.key()} -> std::same_as<typename T::key_type>;
};

template<typename T>
concept info = multiton_info<T> or utils::singleton_info<T>;

template<typename T>
concept owner = requires(T owner){
    typename T::info_type;
    {owner.desc()} -> std::same_as<std::string>;
    {T(std::declval<typename T::info_type>())};
};

template<multiton_info T>
using key_type_t = typename T::key_type;

template<multiton_info T>
using owner_type_t = typename T::owner_type;

namespace multiton{

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

        LOG_ERROR("Multiton instance not found for key {}",key);
        
        throw std::runtime_error("uninitialized multiton"); //TODO:add detailed desc.
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


template<owner owner_type>
[[nodiscard]]
inline auto get(const typename details::multiton_impl<owner_type>::key_type& key) -> owner_type&{
    return details::multiton_impl<owner_type>::get(key);
}

template<typename T>
    requires(!owner<T> && utils::singleton<T>)
inline auto get() -> T&{
    return T::instance();
}

template<info info_type>
inline auto init(const info_type& info) -> void{
    if constexpr (multiton_info<info_type>)
        details::impl_t<info_type>::init(info);
    else get<info_type::owner_type>().init(info);
}

template<multiton_info... info_types>
inline auto init(const info_types&... infos) -> void{
    (init(infos), ...);
}

template<multiton_info info_type>
[[nodiscard]]
inline auto get(const info_type& info) -> owner_type_t<info_type>&{
    using owner_type = owner_type_t<info_type>;
    auto key = info.key();
    if(!details::multiton_impl<owner_type>::contains(key))
        details::multiton_impl<owner_type>::init(info);

    return details::multiton_impl<owner_type>::get(key);
}

template<owner T>
[[nodiscard]]
inline auto desc(const T& owner) -> std::string{
    return owner.desc();
}

}

using namespace multiton;
}