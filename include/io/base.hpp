/**
 * @file io/base.hpp
 * @author Junity
 * @brief IO的基础组件。
 * @details 提供基础的IO处理类，包括裸IO和带键值的IO。
 * 我们把电控中的IO分为两类：裸IO和带键值的IO。
 * @version 0.1
 * @date 2025-11-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstring>
#include <functional>
#include <span>
#include <tuple>
#include <type_traits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include "core/async.hpp"
#include "utils/callback.hpp"
#include "utils/utils.hpp"
#include "utils/concepts.hpp"

namespace roboctrl::io{

/**
 * @brief 只读共享数据缓冲。
 */
using data_ptr = std::shared_ptr<const std::vector<std::byte>>;

/**
 * @brief 只读 byte span。
 */
using byte_span = std::span<const std::byte>;

namespace detail {

template<typename Fn>
awaitable<void> invoke_raw_callback(Fn& fn, data_ptr data) {
    const byte_span bytes{data->data(), data->size()};
    if constexpr (std::same_as<std::invoke_result_t<Fn&, byte_span>, awaitable<void>>) {
        co_await fn(bytes);
    } else {
        fn(bytes);
    }
}

template<typename Fn>
using typed_callback_arg_t = utils::function_arg_t<Fn>;

template<typename Fn>
concept typed_callback_fn =
    utils::package<std::remove_cvref_t<typed_callback_arg_t<Fn>>> &&
    (std::same_as<std::invoke_result_t<Fn&, typed_callback_arg_t<Fn>>, void> ||
     std::same_as<std::invoke_result_t<Fn&, typed_callback_arg_t<Fn>>, awaitable<void>>);

/**
 * @brief 解析 typed payload，并让解析结果覆盖异步回调的完整执行期。
 *
 * `asio::awaitable` 是惰性协程。若直接把 `from_bytes()` 返回的临时对象传给
 * 异步回调，回调真正开始执行前该对象就可能已经析构。这里把解析结果保存在
 * 当前协程帧中，并等待用户回调完成后才释放。
 */
template<typename Arg, typename Fn>
awaitable<void> invoke_typed_callback(Fn& fn, byte_span bytes) {
    using package_type = std::remove_cvref_t<Arg>;
    auto package = utils::from_bytes<package_type>(bytes);

    if constexpr (std::same_as<std::invoke_result_t<Fn&, Arg>, awaitable<void>>) {
        co_await std::invoke(fn, static_cast<Arg>(package));
    } else {
        std::invoke(fn, static_cast<Arg>(package));
    }
}

} // namespace detail

/**
 * @brief 将任意满足 byte_container 的数据拷贝到共享缓冲。
 * @param t 输入缓冲
 */
template<utils::byte_container T>
data_ptr make_shared_from(const T& t) {
    auto res = std::make_shared<std::vector<std::byte>>(t.size());
    if (t.size() > 0)
        std::memcpy(res->data(), t.data(), t.size());
    return res;
}

/**
 * @brief 裸IO组件。
 */
/**
 * @brief 裸数据 IO 基类，仅根据字节流处理事件。
 */
class bare_io_base 
    :public utils::immovable_base, 
    public utils::not_copyable_base
{
public:
    inline bare_io_base(){}
    
    /**
     * @brief 注册字节级别的回调。
     */
    inline void on_data(callback_fn<byte_span> auto fn){
        callback_.add([fn](data_ptr data) mutable -> auto {
            return detail::invoke_raw_callback(fn, std::move(data));
        });
    }

    template<detail::typed_callback_fn Fn>
    requires (!std::same_as<std::remove_cvref_t<utils::function_arg_t<Fn>>,byte_span>)
    inline void on_data(Fn&& fn)
    {
        using Arg = utils::function_arg_t<Fn>;
        static_assert(roboctrl::utils::package<std::remove_cvref_t<Arg>>);

        on_data([fn = std::forward<Fn>(fn)](byte_span bytes) mutable {
            return detail::invoke_typed_callback<Arg>(fn, bytes);
        });
    }

protected:
    /**
     * @brief 分发收到的字节流。
     * @param bytes 接收到的缓冲
     */
    inline void dispatch(byte_span bytes){

        callback_(make_shared_from(bytes));
    }

private:
    callback<data_ptr> callback_;
};

/**
 * @brief 带 key 的 IO 基类，根据键值派发数据。
 * @tparam TK 键类型
 */
template<typename TK>
class keyed_io_base
    :public utils::immovable_base, 
    public utils::not_copyable_base
{
public:
    inline keyed_io_base(){};

    /**
     * @brief 注册指定 key 的回调。
     */
    void on_data(const TK& key,callback_fn<byte_span> auto fn,size_t size = 0){
        const auto [size_it, inserted] = sizes_.try_emplace(key, size);
        if (!inserted) {
            if (size_it->second != 0 && size != 0 && size_it->second != size) {
                throw std::invalid_argument("conflicting payload sizes registered for IO key");
            }
            if (size_it->second == 0) {
                size_it->second = size;
            }
        }
        callbacks_[key].add([fn](data_ptr data) mutable -> auto{
            return detail::invoke_raw_callback(fn, std::move(data));
        });
    }

    /**
     * @brief 注册平凡类型包的回调。
     */
    template<detail::typed_callback_fn Fn>
    requires (!std::same_as<std::remove_cvref_t<utils::function_arg_t<Fn>>,byte_span>)
    inline void on_data(const TK& key,Fn&& fn)
    {
        using Arg = utils::function_arg_t<Fn>;
        static_assert(roboctrl::utils::package<std::remove_cvref_t<Arg>>);
        
        on_data(key,[fn = std::forward<Fn>(fn)](byte_span bytes) mutable {
            return detail::invoke_typed_callback<Arg>(fn, bytes);
        },sizeof(Arg));
    }
protected:
    /**
     * @brief 将数据派发给对应 key 的回调。
     */
    inline void dispatch(const TK& key,byte_span data){
        const auto callback_it = callbacks_.find(key);
        if (callback_it == callbacks_.end()) {
            return;
        }
        const auto size_it = sizes_.find(key);
        if (size_it != sizes_.end() && size_it->second != 0 && size_it->second != data.size()) {
            return;
        }
        callback_it->second(make_shared_from(data));
    }

    inline std::optional<size_t> package_size(const TK& key) const {
        const auto it = sizes_.find(key);
        if (it == sizes_.end() || it->second == 0) {
            return std::nullopt;
        }
        return it->second;
    }

private:
    std::map<TK,callback<data_ptr>> callbacks_;
    std::map<TK,size_t> sizes_;
};

/**
 * @brief 裸 IO 概念，要求具备 send/task 协程接口。
 */
template<typename T>
concept bare_io = std::is_base_of_v<bare_io_base, T> && requires (T t){
    {t.task()} -> std::same_as<awaitable<void>>;
    {t.send(std::declval<byte_span>())} -> std::same_as<awaitable<void>>;
};

/**
 * @brief 带 key 的 IO 概念，要求具备 send/task 协程接口。
 */
template<typename T>
concept keyed_io = std::is_base_of_v<keyed_io_base<typename T::key_type>, T> && requires (T t) {
    typename T::key_type;
    {t.task()} -> std::same_as<awaitable<void>>;
    {t.send(std::declval<typename T::key_type>(),std::declval<byte_span>())} -> std::same_as<awaitable<void>>;
};

/**
 * @brief 数据解析器概念，负责按需消费字节并返回结果。
 */
template<typename T>
concept data_parser = requires (T t,byte_span bytes){
    typename T::data_type;
    {t.parse(bytes)} -> std::same_as<std::size_t>;
    {t.data()} -> std::same_as<typename T::data_type>;
};

/**
 * @brief 组合式解析器，可顺序堆叠多个数据解析单元。
 */
template<data_parser... parser_types>
struct combined_parser{
    template<int N>
    using parser_type = std::tuple_element_t<N,std::tuple<parser_types...>>;

    template<int N>
    using data_type = typename parser_type<N>::data_type;

    /**
     * @brief 依次执行所有解析器。
     * @return 成功消费的字节数，0 表示解析失败
     */
    size_t parse(byte_span data){
        size_t pos = 0;
        bool valid = true;
        std::apply([&](auto&... parser) {
            ([&] {
                if (!valid) {
                    return;
                }
                const size_t consumed = parser.parse(data.subspan(pos));
                if (consumed == 0 || consumed > data.size() - pos) {
                    valid = false;
                    return;
                }
                pos += consumed;
            }(), ...);
        }, parsers);

        return valid ? pos : 0;
    }

    /**
     * @brief 获取指定序号解析器的解析结果。
     */
    template<int N>
    auto data() -> data_type<N>{
        return std::get<N>(parsers).data();
    }

    std::tuple<parser_types...> parsers;
};

/**
 * @brief 固定长度字节解析器。
 * @tparam N 需要解析的字节数
 */
template<int N>
struct nbytes{
    using data_type = std::array<std::byte,N>;

    std::size_t parse(byte_span bytes){
        if(bytes.size() < N){
            return 0;
        }
        std::copy_n(bytes.begin(), N, data_.begin());
        return N;
    }

    data_type data(){
        return data_;
    }

    data_type data_{};
};

/**
 * @brief 平凡结构体解析器。
 */
template<roboctrl::utils::package T>
struct struct_data{
    using data_type = T;

    std::size_t parse(byte_span bytes){
        if(bytes.size() < sizeof(T)){
            return 0;
        }
        std::memcpy(static_cast<void*>(&data_), static_cast<const void*>(bytes.data()), sizeof(T));
        return sizeof(T);
    }

    data_type data(){
        return data_;
    }

    data_type data_{};
};

/**
 * @brief 固定内容匹配解析器。
 */
template<std::byte... bytes>
struct fixed_data{
    using data_type = std::array<std::byte,sizeof...(bytes)>;

    constexpr static std::array<std::byte,sizeof...(bytes)> bytes_data{bytes...};

    std::size_t parse(byte_span data){
        if(data.size() < sizeof...(bytes)){
            return 0;
        }
        
        for(size_t i = 0; i < sizeof...(bytes); ++i)
            if(data[i] != bytes_data[i])
                return 0;

        return sizeof...(bytes);
    }

    data_type data(){
        return bytes_data;
    }
};

/**
 * @brief 吃掉剩余全部数据的解析器。
 */
struct other_all{
    using data_type = byte_span;

    size_t parse(byte_span data){
        data_ = data;
        return data.size();
    }

    data_type data(){
        return data_;
    }

    data_type data_{};
};

/**
 * @brief 发送原始字节流，适配任意裸 IO。
 * @details 示例：
 *
 * ```cpp
 * co_await send(io,data);
 * ```
 */
template<bare_io io_type>
awaitable<void> send(io_type& io,byte_span data){
    co_await io.send(data);
}

/**
 * @brief 发送原始字节流，适配任意裸 IO。
 * @details 示例：
 *
 * ```cpp
 * co_await send<io::udp>("xxx",data);
 * ```
 */
template<bare_io io_type>
awaitable<void> send(const typename io_type::info_type::key_type& key,byte_span data){
    auto& io = roboctrl::get<io_type>(key);
    co_await io.send(data);
}

/**
 * @brief 发送平凡类型数据，会自动转为字节流发送。
 * @details 示例：
 *
 * ```cpp
 * co_await send(io,pkg);
 * ```
 */
template<bare_io io_type,utils::package T>
requires (!std::same_as<T, byte_span>)
awaitable<void> send(io_type& io,T pkg){
    const auto bytes = utils::to_bytes(pkg);
    co_await io.send(byte_span{bytes});
}

/**
 * @brief 发送平凡类型数据，会自动转为字节流发送。
 * @details 示例：
 *
 * ```cpp
 * co_await send<io::udp>("xxx",pkg);
 * ```
 */
template<bare_io io_type,utils::package T>
requires (!std::same_as<T, byte_span>)
awaitable<void> send(typename io_type::info_type::key_type key,T pkg){
    const auto bytes = utils::to_bytes(pkg);
    co_await send<io_type>(key,byte_span{bytes});
}
}
 
