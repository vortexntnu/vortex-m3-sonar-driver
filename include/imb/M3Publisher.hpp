#pragma once
#include <imb/M3Listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <string>
#include <cmath>


namespace m3{
    class M3Publisher : public rclcpp::Node
    {
    private:
        /* data */
        std::vector<uint8_t> data_vector_;
        mutable std::mutex mutex_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        bool packet_ready_;

        /// @brief Publishes the message to the initiated topic
        /// @param message Message to publish (PointCloud2)
        void PublishMessage(sensor_msgs::msg::PointCloud2 message);
        // M3Listener& listener_;
    public:
        /// @brief Default constructor
        M3Publisher();
        
        /// @brief Processes the data from the listener
        void ProcessData();

        /// @brief Creates the listener
        /// @param addr Address to the M3 API
        /// @param port Port to the M3 API (default 20001 / 21001)
        void CreateListener(std::string addr, uint16_t port);

        /// @brief Default destructor
        ~M3Publisher() = default;
    }; 
}