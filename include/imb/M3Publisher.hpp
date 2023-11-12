#pragma once
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <imb/ImbFormat.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace m3{
    class M3Publisher : public rclcpp::Node
    {
    private:
        /* data */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        void PublishMessage(sensor_msgs::msg::PointCloud2 message);
    public:
        M3Publisher();
        ~M3Publisher() = default;
        void ProcessData(m3::imb::DataBody data);
        // void CreateListener(std::string addr, u_int16_t port, M3Publisher& publisher);
    }; 
}