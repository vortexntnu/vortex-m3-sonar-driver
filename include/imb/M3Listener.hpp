#pragma once
#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <imb/M3Publisher.hpp>
namespace m3{
class M3Listener {
public:
    const std::string addr_;
    const u_int16_t port_;
    std::vector<uint8_t>& shared_vector_;
    bool& new_packet_;
    std::mutex& mutex_;
    uint8_t buffer_[1024 * 64]; // How much data to read at a single time (1024 -> 64 * 1024 seems to be working properly)
    sockaddr_in server_addr_;
    int client_socket_;
    M3Listener(std::string addr, u_int16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& new_packet);
    void create_socket();
    void connect_to_sonar();
    void run_listener();
    void stop_listener();
    ~M3Listener();
};
}
