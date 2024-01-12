#pragma once
#include <imb/ImbFormat.hpp>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <mutex>
#include <exception>
#include <vector>
#include <chrono>
#include <thread>

namespace m3{
class M3Listener {
public:
    const std::string addr_;
    const uint16_t port_;
    std::vector<uint8_t>& shared_vector_;
    bool& packet_ready_;
    std::mutex& mutex_;
    uint8_t buffer_[1024 * 64]; // Shared data buffer from the publisher
    sockaddr_in server_addr_;
    int client_socket_;

    /// @brief Default constructor
    /// @param addr Address to the M3 API
    /// @param port Port to the M3 API (default 20001 / 21001)
    /// @param shared_vector Vector to write the data to
    /// @param mutex Shared lock
    /// @param new_packet Boolean to indicate if a new packet is ready
    M3Listener(std::string addr, uint16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& new_header);

    /// @brief Creates the socket with the given address and port
    void create_socket();

    /// @brief Initiates the connection to the M3 API
    void connect_to_sonar();

    /// @brief Starts listening for data from the M3 API
    void run_listener();

    /// @brief Closes the socket
    void stop_listener();

    /// @brief Destructor closing the socket
    ~M3Listener();
};
}
