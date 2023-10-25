#include <imb/TcpServer.hpp>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

namespace m3{
namespace tcp{
TcpServer::TcpServer(const std::string& ipAddress, int port) 
    : port(port), server_fd(0), running(false) {
    setup(ipAddress, port);
}

void TcpServer::setup(const std::string& ipAddress, int port) {
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    int opt = 1;
    // Forcefully attaching socket to the port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    inet_pton(AF_INET, ipAddress.c_str(), &address.sin_addr); // Convert IP address from text to binary form
    address.sin_port = htons(port);

    // Forcefully attaching socket to the port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
}


void TcpServer::run() {
    running = true;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    try {
        while (running) {
            int new_socket;
            if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
                perror("accept");
                continue; // Don't exit, just skip this connection attempt
            }

            std::thread(&TcpServer::handleClient, this, new_socket).detach();
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

void TcpServer::stop() {
    running = false;
    close(server_fd);
}

void TcpServer::handleClient(int client_socket) {
    try {
        uint8_t buffer[1024] = {0};
        std::vector<uint8_t> message;
        bool inMessage = false;

        while (running) {
            ssize_t valread = read(client_socket, buffer, 1024);

            if (valread <= 0) {
                // Connection closed or error
                break;
            }

            for (int i = 0; i < valread; ++i) {
                if (!inMessage) {
                    // Check for message start
                    if (i <= valread - 4 && buffer[i] == 0x80 && buffer[i + 1] == 0x00 && buffer[i + 2] == 0x80 && buffer[i + 3] == 0x00) {
                        inMessage = true;
                        message.clear();
                        i += 3; // Skip the rest of the header
                    }
                } else {
                    message.push_back(buffer[i]);

                    // Add your logic here to check for the end of the message or message completeness
                    // and reset inMessage flag when a whole message is received
                }
            }

            if (inMessage) {
                // If a complete message is detected, add it to the ring buffer
                inMessage = false;
            }
        }

        close(client_socket);
    } catch (const std::exception& e) {
        std::cerr << "Exception in client handling thread: " << e.what() << std::endl;
    }
}
}
}
