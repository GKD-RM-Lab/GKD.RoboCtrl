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
#include <span>
#include <tuple>
#include <type_traits>
#include <map>
#include <memory>
#include <utility>

#include "core/async.hpp"
#include "utils/callback.hpp"
#include "utils/utils.hpp"
#include "utils/concepts.hpp"

namespace roboctrl::io{

/**
 * @brief 数据指针，实际上是一个std::shared_ptr<std::vector<std::byte>>;
 */
using data_ptr = std::shared_ptr<std::vector<std::byte>>;

/**
 * @brief byte span，实际上就是一个std::span<std::byte>;
 */
using byte_span = std::span<std::byte>;

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
            return fn(std::span{data->data(), data->size()});
        });
    }

    template<typename Fn>
    requires (!std::same_as<utils::function_arg_t<Fn>,byte_span>)
    inline void on_data(Fn&& fn)
    {
        using Arg = utils::function_arg_t<Fn>;
        static_assert(roboctrl::utils::package<std::remove_cvref_t<Arg>>);

        on_data([fn = std::forward<Fn>(fn)](byte_span bytes) mutable {
            return fn(utils::from_bytes<std::remove_cvref_t<Arg>>(bytes));
        });
    }

    template<roboctrl::utils::package T>
    /**
     * @brief 发送平凡类型数据。
     */
     requires (!std::same_as<T, byte_span>)
    inline awaitable<void> send(const T& data){
        co_await send(utils::to_bytes(data));
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
    void on_data(const TK& key,callback_fn<byte_span> auto fn){
        callbacks_[key].add([fn](data_ptr data) mutable -> auto{
            return fn(std::span{data->data(),data->size()});
        });
    }

    /**
     * @brief 注册平凡类型包的回调。
     */
    template<typename Fn>
    requires (!std::same_as<utils::function_arg_t<Fn>,byte_span>)
    inline void on_data(const TK& key,Fn&& fn)
    {
        using Arg = utils::function_arg_t<Fn>;
        static_assert(roboctrl::utils::package<std::remove_cvref_t<Arg>>);
        
        on_data(key,[fn = std::forward<Fn>(fn)](byte_span bytes) mutable {
            return fn(utils::from_bytes<std::remove_cvref_t<Arg>>(bytes));
        });
    }
protected:
    /**
     * @brief 将数据派发给对应 key 的回调。
     */
    inline void dispatch(const TK& key,byte_span data){
        auto pd = make_shared_from(data);
        if(callbacks_.contains(key)){
            callbacks_.at(key)(pd);
        }
    }

private:
    std::map<TK,callback<data_ptr>> callbacks_;
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
    {t.parse(std::declval<size_t>(),bytes)} -> std::same_as<std::size_t>;
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
        for(auto& parser : parsers){
            size_t diff  = parser.parse(pos,data);

            if(!diff)
                return false;

            pos += diff;
        }

        return pos;
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

    data_type data_;
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

    data_type data_;
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

    data_type data_;
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
awaitable<void> send(io_type& io,const T& pkg){
    co_await send(io,utils::to_bytes(pkg));
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
awaitable<void> send(const typename io_type::info_type::key_type& key,const T& pkg){
    co_await send(key,utils::to_bytes(pkg));
}
}
 
