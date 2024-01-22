#include <imb/M3Listener.hpp>
#include <fstream>
#include <iosfwd>

#define SYNC_WORD_0 0x80
#define SYNC_WORD_1 0x00

namespace m3{

M3Listener::M3Listener(std::string addr, uint16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& packet_ready)
 : addr_ (addr), port_ (port), shared_vector_ (shared_vector), packet_ready_ (packet_ready), mutex_ (mutex) {

}

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


void M3Listener::connect_to_sonar(){
    while(true){
        std::cout << "[INFO] Attempting to connect to server " << addr_ << " at port " << port_ << std::endl;
        if (connect(client_socket_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == -1) {
            int timeout_delay = 5;
            std::cerr << "Error connecting to server! Trying again in " << timeout_delay << " seconds" << std::endl;
            std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(timeout_delay));
            continue;
        }
        else{
            std::cout << "[INFO] Established connection with server " << addr_ << " at port " << port_ << std::endl;
            break;
        }
        
    }
    
}


void M3Listener::run_listener() {
    std::vector<uint8_t> packet_data; //Holds all the data from each packet-collection
    while (true) {
        // Receive data from the server
        int bytes_read = recv(client_socket_, buffer_, sizeof(buffer_), 0); //Different sizes ranging up to 2^16
        // std::cout << "Received " << bytes_read << " bytes!" << std::endl;
        
        if (bytes_read == -1) { //Cannot read data
            std::cerr << "Error receiving data from the server" << std::endl;
            connect_to_sonar();
        } else if (bytes_read == 0) { // Connection closed by the server
            std::cout << "[INFO] Server closed the connection" << std::endl;
            connect_to_sonar();
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
            if(!is_header || packet_data.size() == 0){
                for (int i = 0; i < bytes_read; i++){ // All data is stored in the vector
                    packet_data.push_back(buffer_[i]);
                }
                continue;
            }
            if(!packet_ready_){
                std::unique_lock<std::mutex> lock(mutex_); // Locks the shared vector (extra protection for thread-safe handling)
                shared_vector_ = packet_data;
                packet_ready_ = true;
                lock.unlock();
            }
            packet_data.clear(); // Resets the vector for a new packet-collection
            for (int i = 0; i < bytes_read; i++){ // All data is stored in the vector
                packet_data.push_back(buffer_[i]);
            }
        }
    }
}


void M3Listener::stop_listener(){
    std::cout << "[INFO] Closing connection to server " << addr_ << " at port " << port_ << std::endl;
    close(client_socket_);
}


M3Listener::~M3Listener(){
    stop_listener();
}

}


