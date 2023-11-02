#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <imb/M3SonarListener.hpp>
#include <exception>
#include <imb/ImbFormat.hpp>
#include <cstdint>
namespace m3{

M3SonarListener::M3SonarListener(std::string addr, u_int16_t port) : addr_ (addr), port_ (port) {

}

void M3SonarListener::create_socket(){
    client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket_ == -1) {
        throw std::runtime_error("Error creating socket");
    }
    // Set up server address information
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    server_addr_.sin_addr.s_addr = inet_addr(addr_.c_str());
}

void M3SonarListener::connect_to_sonar(){
    std::cout << "Connecting to " << addr_ << " at port " << port_ << std::endl;
    if (connect(client_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == -1) {
        close(client_socket_);
        throw std::runtime_error("Error connecting to server");
    }
    std::cout << "Finished connecting";
}

void M3SonarListener::run_listener() {
    // std::cout << std::hex << "11";
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
            // std::cout << std::hex << buffer_ << std::endl;

            // for (int i = 0; i < 1024; i++){
            //     int hexed = int(buffer_[i]);
            //     int next = int(buffer_[i+1]);
            //     if (next == 0x80 && hexed == 0x00){
            //         std::cout << "Synched at " << i << std::endl;
            //     }
            // }
            
            int first = int(buffer_[1]);
            int second = int(buffer_[0]);
            int third = int(buffer_[3]);
            int fourth = int(buffer_[2]);
            int fifth = int(buffer_[5]);
            int sixth = int(buffer_[4]);
            int seventh = int(buffer_[7]);
            int eigth = int(buffer_[6]);

            if (first == 0x80 && second == 0x00 && third == 0x80 && fourth == 0x00 && fifth == 0x80 && sixth == 0x00 && seventh == 0x80 && eigth == 0x00){
                    std::cout << "Hell yeah" << std::endl;
            }
            // uint8_t sync_word = 0x80;
            // uint8_t sync_word_2 = 0x00;
            // for (size_t i=0; i<8; i+=2) {
            //     if(int(buffer_[i]) == sync_word_2 && int(buffer_[i+1]) == sync_word){
            //         std::cout << "Sync!";
            //     }
            //     else{
            //         break;
            //     }
            // }
            // std::cout << "Received data from server" << std::endl;
            //imb::ImbPacketStructure packet(buffer_);

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
