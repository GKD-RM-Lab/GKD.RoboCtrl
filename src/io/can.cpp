#include "io/can.h"
#include "asio/use_awaitable.hpp"
#include "core/async.hpp"
#include "io/base.hpp"
#include "linux/can.h"
#include "utils/utils.hpp"
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <sys/socket.h>

using namespace roboctrl::io;

template <>
struct std::formatter<can_frame> : std::formatter<std::string> {
    auto format(const can_frame& frame, std::format_context& ctx) const {
        std::ostringstream oss;
        oss << std::hex << std::uppercase;
        oss << "CAN ID=0x" << (frame.can_id & CAN_EFF_MASK);

        oss << " [";
        if (frame.can_id & CAN_EFF_FLAG) oss << "EFF ";
        if (frame.can_id & CAN_RTR_FLAG) oss << "RTR ";
        if (frame.can_id & CAN_ERR_FLAG) oss << "ERR ";
        oss << "]";

        oss << " DLC=" << std::dec << static_cast<int>(frame.can_dlc)
            << " DATA=[";

        for (int i = 0; i < frame.can_dlc; ++i) {
            oss << std::format("{:02X}", frame.data[i]);
            if (i + 1 < frame.can_dlc) oss << ' ';
        }
        oss << ']';

        return std::formatter<std::string>::format(oss.str(), ctx);
    }
};

can::can(const can::info_type& info)
    :info_{info},
    keyed_io_base{},
    stream_{roboctrl::io_context()},
    can_name_{info.can_name.data(),info.can_name.length()}
{
    int fd = ::socket(PF_CAN,SOCK_RAW,CAN_RAW);
    if(fd < 0)
        throw std::runtime_error("socket() failed");

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_name_.c_str(), IFNAMSIZ);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
        throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        throw std::runtime_error("bind() failed");

    stream_.assign(fd);

    cf_ = (::can_frame*)std::malloc(sizeof(::can_frame) + 8);
}

can::~can(){
    std::free(cf_);
}

roboctrl::awaitable<void> can::task(){
    while(true){
        size_t pkg_size = co_await stream_.async_read_some(asio::buffer(buffer_),asio::use_awaitable);
        can_frame cf = utils::from_bytes<can_frame>(std::span{buffer_.data(),pkg_size});

        can_id_type id = cf.can_id;
        size_t can_size= cf.can_dlc;

        log_debug("recv can frame: {}",cf);

        dispatch(id,std::span{(std::byte*)cf.data,can_size});
    }
}

roboctrl::awaitable<void> can::send(byte_span frame){
    co_await stream_.async_write_some(asio::buffer(frame),asio::use_awaitable);
}

roboctrl::awaitable<void> can::send(can_id_type id,byte_span data){

    if(data.size() > 8){
        throw std::invalid_argument("payload of can can't > 8");
    }
    
    const auto size = sizeof(can_frame) + data.size();
    cf_->can_id = id;
    cf_->can_dlc = data.size();
    std::memcpy(cf_->data,data.data(),data.size());

    co_await send({(std::byte*)cf_,size});
}