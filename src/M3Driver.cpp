#pragma once
#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <imb/M3Publisher.hpp>


int main(int argc, char *argv[]) {
    // const char *ip_addr = argv[1];
    // std::stringstream strValue;
    // strValue << argv[2];
    // uint16_t port;
    // strValue >> port;

    // m3::M3PclPublisher publisher (argv[3]); // object that publishes the data extracted from the API

    // rclcpp::init(argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<m3::M3Publisher>());
    rclcpp::shutdown();
    
    return 0;
}