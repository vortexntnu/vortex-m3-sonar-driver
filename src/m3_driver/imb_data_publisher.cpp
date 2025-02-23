// M3Publisher.cpp
#include <m3_driver/imb_data_publisher.hpp>
#include <m3_driver/imb_parser.hpp>
#include <thread>
#include <chrono>
#include <spdlog/spdlog.h>

namespace m3 {

M3Publisher::M3Publisher(std::string ip_addr, uint16_t port,
    std::function<void(const pcl::PointCloud<pcl::PointXYZI>&)> publish_callback,
    int callback_time_ms)
    : ip_addr_(std::move(ip_addr))
    , port_(port)
    , publish_callback_(publish_callback)
    , callback_time_ms_(callback_time_ms)
    , data_vector_()
    , packet_ready_(false)
{
    std::thread(&M3Publisher::CreateListener, this, ip_addr_, port_).detach();
    processing_thread_ = std::thread(&M3Publisher::ProcessingLoop, this);
}

M3Publisher::~M3Publisher(){
    if(processing_thread_.joinable()){
        processing_thread_.join();
    }
}

void M3Publisher::ProcessingLoop(){
    while (true) {
        ProcessData();
        std::this_thread::sleep_for(std::chrono::milliseconds(callback_time_ms_));
    }
}

void M3Publisher::ProcessData(){
    try{
        if(!packet_ready_){
            return;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        if(data_vector_.empty()){
            return;
        }
        const uint8_t* start = data_vector_.data();
        m3::imb::ImbPacketStructure packet(start);
        publish_callback_(packet.cloud);
        packet_ready_ = false;
    }
    catch(const std::exception& e){
        std::cerr << e.what() << '\n';
    }
}

void M3Publisher::CreateListener(std::string addr, uint16_t port){
    M3Listener listener(addr, port, data_vector_, mutex_, packet_ready_);
    listener.connect_to_sonar();
    listener.run_listener();
}

} // namespace m3
