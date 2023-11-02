#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <imb/M3SonarListener.hpp>
#include <exception>

namespace m3{

M3SonarListener::M3SonarListener(const char *addr, u_int16_t port) : addr_ (addr), port_ (port) {

}

void M3SonarListener::create_socket(){
    client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket_ == -1) {
        throw std::runtime_error("Error creating socket");
    }
    // Set up server address information
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    server_addr_.sin_addr.s_addr = inet_addr(addr_);
}

void M3SonarListener::connect_to_sonar(){
    std::cout << "Connecting to " << addr_ << " at port " << port_ << std::endl;
    if (connect(client_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == -1) {
        close(client_socket_);
        throw std::runtime_error("Error connecting to server");
    }
}

void M3SonarListener::run_listener() {
    while (true) {
        // Receive data from the server
        int bytes_read = recv(client_socket_, buffer_, sizeof(buffer_), 0);
        if (bytes_read == -1) {
            std::cerr << "Error receiving data from the server" << std::endl;
            break;
        } else if (bytes_read == 0) {
            // Connection closed by the server
            std::cout << "Server closed the connection" << std::endl;
            break;
        } else {
            buffer_[bytes_read] = '\0';
            //std::cout << "Received from server: " << buffer_ << std::endl;
            std::cout << "Received data from server" << std::endl;

        }
    }
}

void M3SonarListener::stop_listener(){
    std::cout << "Closing connection to " << addr_ << " at port " << port_ << std::endl;
    close(client_socket_);
}

}

int main(int argc, char *argv[]) {
    const char *ip_addr = argv[1];

    std::stringstream strValue;
    strValue << argv[2];
    uint16_t port;
    strValue >> port;

    m3::M3SonarListener listener (ip_addr, port);

    listener.create_socket();
    listener.connect_to_sonar();
    listener.run_listener();

    return 0;
}
