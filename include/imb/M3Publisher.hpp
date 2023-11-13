#pragma once
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <imb/ImbFormat.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <mutex>

namespace m3{
    class M3Publisher : public rclcpp::Node
    {
    private:
        /* data */
        std::vector<uint8_t> data_vector_;
        mutable std::mutex mutex_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        mutable bool new_packet_;
        void PublishMessage(sensor_msgs::msg::PointCloud2 message);
        // ~M3Publisher();
        // M3Listener& listener_;
    public:
        M3Publisher();
        void ProcessData();
        void CreateListener(std::string addr, u_int16_t port);
    }; 
}