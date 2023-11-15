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
    bool& new_header_;
    std::mutex& mutex_;
    uint8_t buffer_[1024 * 64]; // Shared data buffer from the publisher
    sockaddr_in server_addr_;
    int client_socket_;
    M3Listener(std::string addr, u_int16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& new_header);
    void create_socket();
    void connect_to_sonar();
    void run_listener();
    void stop_listener();
    ~M3Listener();
};
}
