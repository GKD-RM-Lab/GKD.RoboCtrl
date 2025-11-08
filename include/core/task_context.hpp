#pragma once
#include <asio.hpp>
#include <asio/awaitable.hpp>
#include <chrono>
#include <format>
#include <utility>

#include "asio/executor.hpp"
#include "asio/io_context.hpp"
#include "asio/steady_timer.hpp"
#include "asio/use_awaitable.hpp"
#include "multiton.hpp"
#include "utils/concepts.hpp"

namespace roboctrl::core::task_context{

template<typename T = void>
using awaitable = asio::awaitable<void>;
using duration = std::chrono::steady_clock::duration;

class task_context :
 public utils::immovable_base,
 public utils::not_copyable_base{
    
public:
    using task_type = awaitable<>;

    struct info_type{
        using key_type = int;
        using owner_type = task_context;

        int key_;

        int key() const{
            return key_;
        }

        consteval inline static info_type make(){
            return {.key_ = 0};
        }

        template<int N>
        consteval inline static auto make() -> std::array<info_type,N>{
            return []<std::size_t... I>(std::index_sequence<I...>) -> std::array<info_type,N>{
                return std::array<info_type, N>{info_type{I}...};
            }(std::make_index_sequence<N>());
        }
    };

    inline explicit task_context(info_type _info) : info_(_info){}

    inline void spawn(task_type&& task){
        asio::co_spawn(context_,std::move(task),asio::detached);
    }

    template<typename Fn, typename... Args>
    inline void post(Fn&& fn, Args&&... args) {
        asio::post(context_,
            [fn = std::forward<Fn>(fn),
            args = std::make_tuple(std::decay_t<Args>(std::forward<Args>(args))...)]() mutable {
                std::apply(std::move(fn), std::move(args));
            });
    }


    inline void run(){
        context_.run();
    }

    inline std::string desc()const{
        return std::format("async task context({})",info_.key_);
    }

    inline auto get_executor(){
        return context_.get_executor();
    }

    inline auto asio_context() -> asio::io_context&{
        return context_;
    }

    
private:
    asio::io_context context_;
    info_type info_;
};

static_assert(multiton_info<task_context::info_type>);

inline auto spawn(task_context& tc,task_context::task_type task){
    tc.spawn(std::forward<task_context::task_type>(task));
}

template<typename ...Args>
inline auto post(task_context& tc,Args&&... args)
{
    tc.post(std::forward<Args>(args)...);
}


inline void run(task_context &tc){
    tc.run();
}

inline awaitable<void> yield(){
    co_await asio::post(asio::use_awaitable);
}

inline awaitable<void> wait_for(const duration& duration){
    auto exector = co_await asio::this_coro::executor;
    co_await asio::steady_timer(exector,duration).async_wait(asio::use_awaitable);
}
}

namespace roboctrl{
    using namespace core::task_context;
}