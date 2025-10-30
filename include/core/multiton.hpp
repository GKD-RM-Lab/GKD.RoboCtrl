#pragma once

#include <concepts>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <mutex>
#include <utility>

#include "utils/concepts.hpp"

namespace roboctrl{

template<typename T>
concept info = requires(T info){
    typename T::key_type;
    typename T::owner_type;
    {info.key()} -> std::same_as<typename T::key_type>;
};

template<typename T>
concept owner = requires(T owner){
    typename T::info_type;
    {owner.desc()} -> std::same_as<std::string>;
    {T(std::declval<typename T::info_type>())};
};

template<info T>
using key_type_t = typename T::key_type;

template<info T>
using owner_type_t = typename T::owner_type;

namespace multition{

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
    static auto get(const key_type& key) -> owner_type& {
        std::lock_guard<std::mutex> lock { mutex_ };
        auto it = instances.find(key);
        if (it != instances.end()) {
            return *it->second;
        }
        
        throw std::runtime_error("uninitialized multition"); //TODO:add detailed desc.
    };
};

template<info info_type>
using impl_t = multiton_impl<owner_type_t<info_type>>;

template <typename owner_type>
std::mutex multiton_impl<owner_type>::mutex_ {};

template <typename owner_type>
std::unordered_map<
    typename multiton_impl<owner_type>::key_type, 
    std::unique_ptr<owner_type>
> multiton_impl<owner_type>::instances {};
}

template<info info_type>
inline auto init(const info_type& info) -> void{
    details::impl_t<info_type>::init(info);
}

template<owner owner_type>
inline auto get(const typename details::multiton_impl<owner_type>::key_type& key) -> owner_type&{
    return details::multiton_impl<owner_type>::get(key);
}

template<owner T>
inline auto desc(T owner) -> std::string{
    return owner.desc();
}

}

using namespace multition;
}