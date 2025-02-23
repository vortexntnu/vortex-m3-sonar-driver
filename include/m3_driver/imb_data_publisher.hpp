#pragma once
#include <vector>
#include <mutex>
#include <functional>
#include <cstdint>
#include <string>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "m3_driver/imb_data_listener.hpp"
#include <exception>
#include <chrono>
#include <iostream>

namespace m3 {

class M3Publisher {
public:
    // publish_callback: function to publish the point cloud (pcl::PointCloud<pcl::PointXYZI>)
    // callback_time_ms: processing interval in milliseconds
    M3Publisher(std::string ip_addr, uint16_t port,
                std::function<void(const pcl::PointCloud<pcl::PointXYZI>&)> publish_callback,
                int callback_time_ms);
    ~M3Publisher();

private:
    std::string ip_addr_;
    uint16_t port_;
    std::function<void(const pcl::PointCloud<pcl::PointXYZI>&)> publish_callback_;
    int callback_time_ms_;
    std::vector<uint8_t> data_vector_;
    bool packet_ready_;
    std::mutex mutex_;
    std::thread processing_thread_;

    void ProcessData();
    void ProcessingLoop();
    void CreateListener(std::string addr, uint16_t port);
};

} // namespace m3
