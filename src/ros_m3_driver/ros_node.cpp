#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <spdlog/spdlog.h>

#include <m3_driver/imb_data_publisher.hpp>

class M3RosNode : public rclcpp::Node {
public:
    M3RosNode() : Node("m3_ros_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("m3_point_cloud", 10);
        auto publish_callback = [this](const pcl::PointCloud<pcl::PointXYZI>& cloud) {
            sensor_msgs::msg::PointCloud2 ros_msg;
            pcl::toROSMsg(cloud, ros_msg);
            ros_msg.header.frame_id = "m3_sonar";
            ros_msg.header.stamp = this->now();
            publisher_->publish(ros_msg);
            spdlog::info("Published point cloud with {} points.", cloud.points.size());
        };
        std::string ip_addr = "172.29.208.1";
        uint16_t port = 20001;
        int callback_time_ms = 100;
        m3_publisher_ = std::make_shared<m3::M3Publisher>(ip_addr, port, publish_callback, callback_time_ms);
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<m3::M3Publisher> m3_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<M3RosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
