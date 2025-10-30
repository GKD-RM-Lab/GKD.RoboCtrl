#include "io/can.h"
#include "asio/use_awaitable.hpp"
#include "core/task_context.hpp"
#include "io/base.hpp"
#include "linux/can.h"
#include "utils/utils.hpp"
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <sys/socket.h>

using namespace roboctrl::io;

can_io::can_io(const can_io::info_type& info)
    :info_{info},
    keyed_io_base{info.context},
    stream_{info.context.asio_context()}
{
    int fd = ::socket(PF_CAN,SOCK_RAW,CAN_RAW);
    if(fd < 0)
        throw std::runtime_error("socket() failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, info.can_name.c_str(), IFNAMSIZ);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
        throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        throw std::runtime_error("bind() failed");

    stream_.assign(fd);
}

roboctrl::awaitable<void> can_io::task(){
    while(true){
        co_await stream_.async_read_some(asio::buffer(buffer_),asio::use_awaitable);
        can_frame cf = utils::from_bytes<can_frame>(buffer_);

        can_id_type id = cf.can_id;
        size_t size= cf.can_dlc;

        dispatch(id,std::span<std::byte>((std::byte*)cf.data,size));
    }
}

roboctrl::awaitable<void> can_io::send(byte_span frame){
    if(frame.size() != sizeof(::can_frame)){
        co_return;
    }

    co_await stream_.async_write_some(asio::buffer(frame));
}

roboctrl::awaitable<void> can_io::send(can_id_type id,byte_span data){

    if(data.size() > 8){
        throw std::invalid_argument("payload of can can't > 8");
    }
    
    const auto size = sizeof(can_frame) + data.size();
    can_frame *cf = (can_frame*)std::malloc(size);
    cf->can_id = id;
    cf->can_dlc = data.size();
    std::memcpy(cf->data,data.data(),data.size());

    co_await send({(std::byte*)cf,size});

    std::free(cf);
}