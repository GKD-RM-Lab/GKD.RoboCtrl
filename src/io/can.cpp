#include "io/can.h"
#include "asio/use_awaitable.hpp"
#include "core/async.hpp"
#include "io/base.hpp"
#include "linux/can.h"
#include "utils/utils.hpp"
#include <cstddef>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

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

        const auto length = std::min<int>(frame.can_dlc, CAN_MAX_DLEN);
        for (int i = 0; i < length; ++i) {
            oss << std::format("{:02X}", frame.data[i]);
            if (i + 1 < length) oss << ' ';
        }
        oss << ']';

        return std::formatter<std::string>::format(oss.str(), ctx);
    }
};

can::can(const can::info_type& info)
    :info_{info},
    keyed_io_base{},
    stream_{roboctrl::io_context()},
    write_queue_{[this](byte_span data) -> awaitable<void> {
        co_await asio::async_write(stream_, asio::buffer(data), asio::use_awaitable);
    }},
    interface_name_{info.interface_name.data(),info.interface_name.length()}
{
    int fd = ::socket(PF_CAN,SOCK_RAW,CAN_RAW);
    if(fd < 0)
        throw std::runtime_error("socket() failed");

    if (interface_name_.empty() || interface_name_.size() >= IFNAMSIZ) {
        ::close(fd);
        throw std::invalid_argument("invalid CAN interface name");
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        ::close(fd);
        throw std::runtime_error("ioctl(SIOCGIFINDEX) failed");
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ::close(fd);
        throw std::runtime_error("bind() failed");
    }

    stream_.assign(fd);

    log_info("Can io {} created on {}", info.name, info.interface_name);
}

void can::start() {
    if (started_) {
        return;
    }
    started_ = true;
    roboctrl::spawn(task());
}

roboctrl::awaitable<void> can::task(){
    while(true){
        size_t pkg_size = co_await stream_.async_read_some(asio::buffer(buffer_),asio::use_awaitable);
        if (pkg_size != sizeof(can_frame)) {
            log_warn("drop malformed CAN frame with {} bytes", pkg_size);
            continue;
        }
        can_frame cf = utils::from_bytes<can_frame>(std::span{buffer_.data(),pkg_size});

        can_id_type id = cf.can_id;
        size_t can_size= cf.can_dlc;

        if (can_size > CAN_MAX_DLEN || (id & (CAN_RTR_FLAG | CAN_ERR_FLAG))) {
            log_warn("drop invalid CAN frame: {}", cf);
            continue;
        }

        //log_debug("recv can frame: {}",cf);

        dispatch(id,std::span{(std::byte*)cf.data,can_size});
    }
}

roboctrl::awaitable<void> can::send(can_id_type id, byte_span data) {

    if (data.size() > CAN_MAX_DLEN) {
        throw std::invalid_argument("CAN payload cannot exceed 8 bytes");
    }
    if (id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) {
        throw std::invalid_argument("CAN data frame ID cannot contain RTR or ERR flags");
    }

    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = static_cast<decltype(frame.can_dlc)>(data.size());
    std::memcpy(frame.data, data.data(), data.size());

    log_debug("send can frame: {}", frame);

    co_await write_queue_.send_latest(
        id,
        std::span<const std::byte>{reinterpret_cast<const std::byte*>(&frame), sizeof(frame)});
}
