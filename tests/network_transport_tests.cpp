#include "device/aim_link.hpp"
#include "device/remote_logger.hpp"

#include <iostream>
#include <stdexcept>

namespace {
using namespace roboctrl;
using namespace std::chrono_literals;
void require(bool value) {
    if (!value) throw std::runtime_error("network transport test failed");
}

std::array<std::byte, 14> aim_packet(std::uint8_t header) {
    std::array<std::byte, 14> packet{};
    packet[0] = std::byte{header};
    device::network_protocol::write_float(packet, 1, 1.0f);
    device::network_protocol::write_float(packet, 5, -0.5f);
    packet[9] = std::byte{1};
    return packet;
}

awaitable<void> exercise(io::udp_server& server, asio::ip::udp::socket& peer,
                         asio::ip::udp::socket& stranger, bool& completed) {
    auto& aim = get<device::aim_link>("test_aim");
    auto& second_aim = get<device::aim_link>("test_second_aim");
    auto& nav = get<device::navigation_link>("test_nav");
    auto& logger = get<device::remote_logger>("test_log");
    require(!aim.target() && !nav.command());
    const auto destination = server.local_endpoint();
    const auto packet = aim_packet(0x6a);

    co_await stranger.async_send_to(asio::buffer(packet), destination, asio::use_awaitable);
    co_await wait_for(5ms);
    require(!aim.target()); // Same valid bytes from the wrong source port are ignored.

    co_await peer.async_send_to(asio::buffer(packet), destination, asio::use_awaitable);
    co_await wait_for(5ms);
    const auto target = aim.target();
    require(target && target->yaw == 1.0f && target->pitch == -0.5f && target->fire);
    require(!second_aim.target() && !nav.command()); // Header dispatch shares one peer.
    const auto second = aim_packet(0x6b);
    co_await peer.async_send_to(asio::buffer(second), destination, asio::use_awaitable);
    std::array<std::byte, 9> navigation{};
    navigation[0] = std::byte{0x37};
    device::network_protocol::write_float(navigation, 1, 2.0f);
    device::network_protocol::write_float(navigation, 5, -1.0f);
    co_await peer.async_send_to(asio::buffer(navigation), destination, asio::use_awaitable);
    co_await wait_for(5ms);
    require(second_aim.target() && nav.command() && nav.command()->vx == 2.0f);

    // Two queued sends retain separate owned payloads until the socket finishes.
    co_await aim.send_posture(1.0f, -0.5f, true);
    co_await nav.send_status(0.5f, 0.75f, true);
    std::array<std::byte, 512> received{};
    asio::ip::udp::endpoint from;
    auto size = co_await peer.async_receive_from(asio::buffer(received), from, asio::use_awaitable);
    require(size == 10 && received[0] == std::byte{0x6a} && received[9] == std::byte{1} &&
            device::network_protocol::read_float(received, 1) == 1.0f && from == destination);
    size = co_await peer.async_receive_from(asio::buffer(received), from, asio::use_awaitable);
    require(size == 10 && received[0] == std::byte{0x37} &&
            device::network_protocol::read_float(received, 5) == 0.75f);
    co_await logger.push_value("test.yaw", 1.0);
    size = co_await peer.async_receive_from(asio::buffer(received), from, asio::use_awaitable);
    require(size == 31 && received[0] == std::byte{16} && received[2] == std::byte{0} &&
            received[16] == std::byte{15} && received[18] == std::byte{1});

    // Invalid packets must not refresh the previous valid fire request.
    auto invalid = packet;
    invalid[9] = std::byte{2};
    for (int i = 0; i < 5; ++i) {
        co_await peer.async_send_to(asio::buffer(invalid), destination, asio::use_awaitable);
        co_await wait_for(25ms);
    }
    require(!aim.target() && !second_aim.target() && !nav.command());

    server.stop();
    peer.close();
    stranger.close();
    completed = true;
}

awaitable<void> watchdog(bool& completed) {
    co_await wait_for(2s);
    if (!completed) async::stop();
}

std::weak_ptr<int> enqueue_temporary_callback(bool& completed) {
    // Both the callback object and caller-owned bytes disappear before run().
    callback<std::shared_ptr<int>> temporary;
    temporary.add([&completed](std::shared_ptr<int> value) -> awaitable<void> {
        co_await yield();
        completed = *value == 42;
    });
    auto value = std::make_shared<int>(42);
    const std::weak_ptr<int> weak = value;
    temporary(std::move(value));
    return weak;
}
} // namespace

int main() {
    using namespace roboctrl;
    try {
        init(io::udp_server::info_type{"test_udp", "127.0.0.1", 0});
        auto& server = get<io::udp_server>("test_udp");
        asio::ip::udp::socket peer{executor(), {asio::ip::make_address("127.0.0.1"), 0}};
        asio::ip::udp::socket stranger{executor(), {asio::ip::make_address("127.0.0.1"), 0}};
        const auto port = peer.local_endpoint().port();
        init(device::aim_link::info_type{"test_aim", "test_udp", "127.0.0.1", port, 0x6a, 100});
        init(device::aim_link::info_type{"test_second_aim", "test_udp", "127.0.0.1", port, 0x6b, 100});
        init(device::navigation_link::info_type{"test_nav", "test_udp", "127.0.0.1", port, 100});
        init(device::remote_logger::info_type{"test_log", "test_udp", "127.0.0.1", port});
        connect_all<io::udp_server>();
        connect_all<device::aim_link>();
        connect_all<device::navigation_link>();
        connect_all<device::remote_logger>();
        start_all<device::aim_link>();
        start_all<device::navigation_link>();
        start_all<device::remote_logger>();
        start_all<io::udp_server>();
        bool completed = false;
        bool callback_completed = false;
        const auto callback_value = enqueue_temporary_callback(callback_completed);
        require(!callback_value.expired());
        spawn(exercise(server, peer, stranger, completed));
        spawn(watchdog(completed));
        async::run();
        require(completed && callback_completed && callback_value.expired());
        std::cout << "network loopback and callback lifetime tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
