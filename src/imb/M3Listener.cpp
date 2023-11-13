
#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <imb/M3Listener.hpp>
#include <exception>
#include <imb/ImbFormat.hpp>
#include <cstdint>
#include <vector>

#define SYNC_WORD_0 0x80
#define SYNC_WORD_1 0x00

namespace m3{

/// @brief Default constructor
/// @param addr Address to the M3 API
/// @param port Port to the M3 API (default 20001 / 21001)
/// @param shared_vector Vector to write the data to
/// @param mutex Shared lock
/// @param new_packet Boolean to indicate if a new packet is ready
M3Listener::M3Listener(std::string addr, u_int16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& new_packet)
 : addr_ (addr), port_ (port), shared_vector_ (shared_vector), new_packet_ (new_packet), mutex_ (mutex) {

}

/// @brief Creates the socket
void M3Listener::create_socket(){
    client_socket_ = socket(AF_INET, SOCK_STREAM, 0); //Initialize socket
    if (client_socket_ == -1) {
        throw std::runtime_error("Error creating socket");
    }
    // Set up server address information
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    server_addr_.sin_addr.s_addr = inet_addr(addr_.c_str());
}

/// @brief Initiates the connection to the sonar
void M3Listener::connect_to_sonar(){
    std::cout << "[INFO] Attempting to connect to server " << addr_ << " at port " << port_ << std::endl;
    if (connect(client_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == -1) {
        close(client_socket_);
        throw std::runtime_error("Error connecting to server");
    }
    std::cout << "[INFO] Established connection with server " << addr_ << " at port " << port_ << std::endl;
}

/// @brief Starts listening for data from the M3 API
void M3Listener::run_listener() {
    std::vector<uint8_t> packet_data; //Holds all the data from each packet-collection
    bool first_iter = true; // Used to ignore the first iteration, as the packet is not complete
    while (true) {
        // Receive data from the server
        int bytes_read = recv(client_socket_, buffer_, sizeof(buffer_), 0); //Different sizes ranging up to 2^16
        // std::cout << "Received " << bytes_read << " bytes!" << std::endl;
        
        if (bytes_read == -1) { //Cannot read data
            std::cerr << "Error receiving data from the server" << std::endl;
            break;
        } else if (bytes_read == 0) { // Connection closed by the server
            std::cout << "[INFO] Server closed the connection" << std::endl;
            break;
        } else {
            buffer_[bytes_read] = '\0'; // Make sure the buffer is getting ended (should not be necessary)

            bool is_header = (int(buffer_[0]) == SYNC_WORD_1 // Check for synchronization word to find header packet
                && int(buffer_[2]) == SYNC_WORD_1
                && int(buffer_[4]) == SYNC_WORD_1
                && int(buffer_[6]) == SYNC_WORD_1
                && int(buffer_[1]) == SYNC_WORD_0
                && int(buffer_[3]) == SYNC_WORD_0
                && int(buffer_[5]) == SYNC_WORD_0
                && int(buffer_[7]) == SYNC_WORD_0
            );
            // std::cout << "Header: " << is_header << std::endl;
            if (is_header){ //Packet contains header -> create the object and send it to publisher
                if(!first_iter && !new_packet_){
                    std::unique_lock<std::mutex> lock(mutex_); // Locks the shared vector (extra protection for thread-safe handling)
                    shared_vector_ = packet_data;
                    new_packet_ = true;
                    std::cout << "[INFO] New packet ready!" << std::endl;

                    lock.unlock();

                    packet_data.clear(); // Resets the vector for a new packet-collection

                }
                else{
                    first_iter = false; //Ignores the first iteration, as the packet is not complete
                }
                
            }
            for (uint8_t byte : buffer_){ // All data is stored in the vector
                    packet_data.push_back(byte);
            }
        }
    }
}

/// @brief Closes the socket
void M3Listener::stop_listener(){
    std::cout << "[INFO] Closing connection to " << addr_ << " at port " << port_ << std::endl;
    close(client_socket_);
}

/// @brief Destructor closing the socket
M3Listener::~M3Listener(){
    stop_listener();
}

}


