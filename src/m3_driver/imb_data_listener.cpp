#include <m3_driver/imb_data_listener.hpp>
#include <fstream>
#include <iosfwd>
#include <boost/asio.hpp>
#include <spdlog/spdlog.h>
#include <thread>

#define SYNC_WORD_0 0x80
#define SYNC_WORD_1 0x00

namespace m3 {

using boost::asio::ip::tcp;

M3Listener::M3Listener(std::string addr, uint16_t port, std::vector<uint8_t>& shared_vector, std::mutex& mutex, bool& packet_ready)
    : addr_(std::move(addr))
    , port_(port)
    , shared_vector_(shared_vector)
    , packet_ready_(packet_ready)
    , mutex_(mutex)
    , io_context_()
    , socket_(io_context_)
{
}

void M3Listener::connect_to_sonar(){
    tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(addr_, std::to_string(port_));
    while(true){
        spdlog::info("Attempting to connect to server {} at port {}", addr_, port_);
        boost::system::error_code ec;
        boost::asio::connect(socket_, endpoints, ec);
        if(ec){
            int timeout_delay = 5;
            spdlog::error("Error connecting to server! Trying again in {} seconds. Error: {}", timeout_delay, ec.message());
            std::this_thread::sleep_for(std::chrono::seconds(timeout_delay));
            continue;
        }
        else{
            spdlog::info("Established connection with server {} at port {}", addr_, port_);
            break;
        }
    }
}

void M3Listener::run_listener() {
    std::vector<uint8_t> packet_data;
    uint8_t buffer[65536]; // Adjust size as needed
    while (true) {
        boost::system::error_code ec;
        size_t bytes_read = socket_.read_some(boost::asio::buffer(buffer), ec);
        if (ec) {
            spdlog::error("Error receiving data from the server: {}", ec.message());
            connect_to_sonar();
            continue;
        } else if (bytes_read == 0) {
            spdlog::info("Server closed the connection");
            connect_to_sonar();
            continue;
        } else {
            bool is_header = (bytes_read >= 8 &&
                int(buffer[0]) == SYNC_WORD_1 &&
                int(buffer[2]) == SYNC_WORD_1 &&
                int(buffer[4]) == SYNC_WORD_1 &&
                int(buffer[6]) == SYNC_WORD_1 &&
                int(buffer[1]) == SYNC_WORD_0 &&
                int(buffer[3]) == SYNC_WORD_0 &&
                int(buffer[5]) == SYNC_WORD_0 &&
                int(buffer[7]) == SYNC_WORD_0);
            if(!is_header || packet_data.empty()){
                packet_data.insert(packet_data.end(), buffer, buffer + bytes_read);
                continue;
            }
            if(!packet_ready_){
                std::unique_lock<std::mutex> lock(mutex_);
                shared_vector_ = packet_data;
                packet_ready_ = true;
            }
            packet_data.clear();
            packet_data.insert(packet_data.end(), buffer, buffer + bytes_read);
        }
    }
}

void M3Listener::stop_listener(){
    spdlog::info("Closing connection to server {} at port {}", addr_, port_);
    boost::system::error_code ec;
    socket_.shutdown(tcp::socket::shutdown_both, ec);
    socket_.close(ec);
}

M3Listener::~M3Listener(){
    stop_listener();
}

} // namespace m3
