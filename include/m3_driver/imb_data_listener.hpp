#pragma once
#include <m3_driver/imb_parser.hpp>
#include <sstream>
#include <boost/asio.hpp>
#include <mutex>
#include <exception>
#include <vector>
#include <chrono>
#include <thread>

namespace m3 {
class M3Listener {
public:
    const std::string addr_;
    const uint16_t port_;
    std::vector<uint8_t>& shared_vector_;
    bool& packet_ready_;
    std::mutex& mutex_;

    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket socket_;

    M3Listener(std::string addr, uint16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& new_header);

    void connect_to_sonar();
    void run_listener();
    void stop_listener();

    ~M3Listener();
};
}
