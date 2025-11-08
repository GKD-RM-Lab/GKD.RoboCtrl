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
#include <utility>

#include "core/task_context.hpp"
#include "utils/callback.hpp"
#include "utils/utils.hpp"
#include "utils/concepts.hpp"

namespace roboctrl::io{

using data_ptr = std::shared_ptr<std::vector<std::byte>>;
using byte_span = std::span<std::byte>;

template<utils::byte_container T>
data_ptr make_shared_from(const T& t) {
    auto res = std::make_shared<std::vector<std::byte>>(t.size());
    if (t.size() > 0)
        std::memcpy(res->data(), t.data(), t.size());
    return res;
}

class bare_io_base 
    :public utils::immovable_base, 
    public utils::not_copyable_base
{
public:
    inline bare_io_base(task_context& tc):tc_{tc}{}

    inline void dispatch(byte_span bytes){

        callback_(tc_,make_shared_from(bytes));
    }

    
    inline void on_data(callback_fn<byte_span> auto fn){
        callback_.add([fn](data_ptr data) -> auto{
            return fn(std::span{data->data(),data->size()});
        });
    }

    template<roboctrl::utils::package T>
    inline void on_data(callback_fn<T> auto fn){
        on_data([fn](byte_span bytes) -> auto{
            return fn(utils::from_bytes<T>(bytes));
        });
    }

    template<roboctrl::utils::package T>
    inline awaitable<void> send(const T& data){
        co_await send(utils::to_bytes(data));
    }

private:
    callback<data_ptr> callback_;
    task_context &tc_;
};

template<typename TK>
class keyed_io_base
    :public utils::immovable_base, 
    public utils::not_copyable_base
{
public:
    inline keyed_io_base(task_context& tc):tc_{tc}{};

    inline void dispatch(const TK& key,byte_span data){
        auto pd = make_shared_from(data);
        if(callbacks_.contains(key)){
            callbacks_.at(key)(tc_,pd);
        }
    }

    void on_data(const TK& key,callback_fn<data_ptr> auto fn){
        callbacks_[key].add(fn);
    }

    template<utils::package T>
    void on_data(const TK& key,callback_fn<T> auto fn){
        on_data(key,[fn](data_ptr bytes){
            return fn(utils::from_bytes<T>(*bytes));
        });
    }

private:
    task_context& tc_;
    std::map<TK,callback<data_ptr>> callbacks_;
};

template<typename T>
concept bare_io = std::is_base_of_v<bare_io_base, T> && requires (T t,byte_span data){
    {t.task()} -> std::same_as<awaitable<void>>;
    {t.send(data)} -> std::same_as<awaitable<void>>;
};

template<typename T>
concept data_parser = requires (T t,byte_span bytes){
    typename T::data_type;
    {t.parse(std::declval<size_t>(),bytes)} -> std::same_as<std::size_t>;
    {t.data()} -> std::same_as<typename T::data_type>;
};

template<data_parser... parser_types>
struct combined_parser{
    template<int N>
    using parser_type = std::tuple_element_t<N,std::tuple<parser_types...>>;

    template<int N>
    using data_type = typename parser_type<N>::data_type;

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

    template<int N>
    auto data() -> data_type<N>{
        return std::get<N>(parsers).data();
    }

    std::tuple<parser_types...> parsers;
};

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

template<bare_io io_type>
awaitable<void> send(io_type& io,byte_span data){
    co_await io.send(data);
}

template<bare_io io_type,utils::package T>
awaitable<void> send(io_type& io,const T& pkg){
    co_await send(io,utils::to_bytes(pkg));
}
}
 