// listener.hpp
#pragma once
#include <m3_driver/imb_parser.hpp>
#include <sstream>
#include <boost/asio.hpp>
#include <mutex>
#include <condition_variable>
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
    std::mutex& mutex_;
    std::condition_variable& cond_var_;

    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket socket_;

    M3Listener(std::string addr, uint16_t port, std::vector<uint8_t>& shared_vector,
               std::mutex& mutex, std::condition_variable& cond_var);

    void connect_to_sonar();
    void run_listener();
    void stop_listener();

    ~M3Listener();
};
}
