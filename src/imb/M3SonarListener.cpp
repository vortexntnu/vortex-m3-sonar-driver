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
#include <vector>

#define SYNC_WORD_0 0x80
#define SYNC_WORD_1 0x00

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
    std::cout << "Finished connecting" << std::endl;
}

void M3SonarListener::run_listener() {

    //Holds all the data from each packet-collection
    std::vector<uint8_t> packet_data;
    bool first_iter = true;
    while (true) {
        // Receive data from the server
        int bytes_read = recv(client_socket_, buffer_, sizeof(buffer_), 0); //Different sizes ranging up to 2^16
        // std::cout << "Received " << bytes_read << " bytes!" << std::endl;
        
        if (bytes_read == -1) { //Cannot read data
            std::cerr << "Error receiving data from the server" << std::endl;
            break;
        } else if (bytes_read == 0) { // Connection closed by the server
            std::cout << "Server closed the connection" << std::endl;
            break;
        } else {
            buffer_[bytes_read] = '\0'; // Make sure the buffer is getting ended

            

            bool is_header = (int(buffer_[0]) == SYNC_WORD_1 //Check for synchronization word
                && int(buffer_[2]) == SYNC_WORD_1
                && int(buffer_[4]) == SYNC_WORD_1
                && int(buffer_[6]) == SYNC_WORD_1
                && int(buffer_[1]) == SYNC_WORD_0
                && int(buffer_[3]) == SYNC_WORD_0
                && int(buffer_[5]) == SYNC_WORD_0
                && int(buffer_[7]) == SYNC_WORD_0
            );
            if (is_header){ //Packet contains header -> create the object and send it to publisher
                if(!first_iter){
                    const uint8_t* start = &packet_data[0]; // grabs the pointer to the first element in the vector which now contains the whole packet
                    // std::cout << bytes_read << std::endl;
                    imb::PacketHeader packet_header(start);

                    imb::DataHeader data_header(start + sizeof(imb::PacketHeader));

                    imb::DataBody data_body(start + sizeof(imb::PacketHeader) + sizeof(imb::DataHeader), data_header.nNumBeams, data_header.nNumImageSample, packet_header.dataType);
                    // std::memcpy(&ph, buffer_, sizeof(imb::PacketHeader));
                    
                    // std::cout << "Data body size: " << packet_header.packetBodySize;
                    // std::cout << "\nAltitude : " << data_header.bSoundSpeedSource;
                    // std::cout << "\nData header size : " << sizeof(imb::DataHeader);
                    // std::cout << "\nSize : " << packet_data.size() << std::endl;

                    // std::cout << data_body.complexData << std::endl;


                    // std::cout << "\nCapacity : " << packet_data.capacity();

                    packet_data.clear(); // clear the data

                    // std::cout << "Received header packet with size "<< bytes_read << std::endl;
                }
                else{
                    first_iter = false; //Continue after this iter
                }
                
            }
            else { //Packet contains data
                

                //std::cout << "Received illegal packet with size " << bytes_read << std::endl;
                // std::cout << "Not header with size " << bytes_read << std::endl;
            }

            for (uint8_t byte : buffer_){ // add all data from the buffer to the vector
                    packet_data.push_back(byte);
            }
            
            // if (bytes_read > 1024 * 8){
            //     // std::cout << bytes_read << std::endl;
            // }
            
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

int main(int, char *argv[]) {
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
