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
    M3Publisher& publisher_;
    int client_socket_;
    uint8_t buffer_[1024 * 64]; // How much data to read at a single time (1024 -> 64 * 1024 seems to be working properly)
    sockaddr_in server_addr_;
    M3Listener(std::string addr, u_int16_t port, M3Publisher& publisher);
    void create_socket();
    void connect_to_sonar();
    void run_listener();
    void stop_listener();
};
}
