
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <imb/ImbFormat.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <mutex>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <imb/M3Listener.hpp>

using namespace std::chrono_literals;
namespace m3{
    M3Publisher::M3Publisher() : Node("M3Publisher"){

        // Define the quality of service profile for publisher to match the other publishers and subscribers
        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        // qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

        this->declare_parameter("ip_addr", rclcpp::PARAMETER_STRING);
        this->declare_parameter("port", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("topic", rclcpp::PARAMETER_STRING);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("topic").as_string().c_str(), 10);
        data_vector_ = std::vector<uint8_t>();
        new_header_ = false;
        std::thread(&M3Publisher::CreateListener, this, this->get_parameter("ip_addr").as_string(), this->get_parameter("port").as_int()).detach();
        timer_ = this->create_wall_timer(100ms, std::bind(&M3Publisher::ProcessData, this));
    }
    void M3Publisher::ProcessData() {
        try{
            // std::cout << new_header_ << std::endl;
            if(new_header_){
                std::unique_lock<std::mutex> lock(mutex_); // Thread-safe handling
                const uint8_t* start = &data_vector_[0]; // Grabs the pointer to the first element in the vector
                // imb::ImbPacketStructure packet(start); // TODO: - fix packet structure

                // Cast data to the correct types
                imb::PacketHeader packet_header(start);
                imb::DataHeader data_header(start + sizeof(imb::PacketHeader));
                imb::DataBody data_body(start + sizeof(imb::PacketHeader) + sizeof(imb::DataHeader),
                    data_header.nNumBeams, data_header.nNumImageSample, packet_header.dataType);

                // std::cout << "Speed of sound " << data_header.fSoundSpeed << std::endl;
                // std::vector<pcl::PointXYZI> points_for_cloud = std::vector<pcl::PointXYZI>();
                //Inintialize the pointcloud
                pcl::PointCloud<pcl::PointXYZI> cloud;
                cloud.height = 1;
                cloud.width = data_header.nNumBeams * data_header.nNumImageSample;
                cloud.is_dense = false;
                cloud.points.resize(cloud.height * cloud.width);

                for(size_t i = 0; i < data_header.nNumBeams; i++){
                    float beam_angle = data_header.fBeamList[i]; //Current beam angle
                    for(size_t j = 0; j < data_header.nNumImageSample; j++){
                        pcl::PointXYZI point;
                        float dist_to_point = ((data_header.fSWST - data_header.fTXWST) + data_header.fImageSampleInterval * j) 
                            * data_header.fSoundSpeed / 2; // Formula given in the IMB format documentation
                        point.x = -dist_to_point * sin(beam_angle * M_PI / 180.0);
                        point.y = dist_to_point * cos(beam_angle * M_PI / 180.0);
                        point.z = 0;
                        float intensity = data_body.complexData(i, j).real();
                        if(intensity > 0){
                            point.intensity = intensity;
                        }
                        else{
                            point.intensity = 0;
                        }
                        cloud.points[i * data_header.nNumImageSample + j] = point;
                    }
                }
                // std::cout << "[INFO] Cloud Size: " << cloud.points.size() << std::endl;
                // Convert the pointcloud to a ROS message
                sensor_msgs::msg::PointCloud2 message;
                pcl::toROSMsg(cloud, message);
                message.header.frame_id = "m3_sonar";
                message.header.stamp = rclcpp::Time(data_header.dwTimeSec, data_header.dwTimeMillisec); // Timestamp from sonar data
                PublishMessage(message);
                

                // TODO - find a way to create the pointcloud2 message directly as sensor_msg
                // std::vector<sensor_msgs::msg::PointField> points = std::vector<sensor_msgs::msg::PointField>();
                // sensor_msgs::msg::PointCloud2 message;
                // message.fields = points;

                new_header_ = false; // Ready to receive new packet
                lock.unlock();
            }
        }
        catch(const std::exception& e){
            std::cerr << e.what() << '\n';
        }
       
    }
    /// @brief Publishes the message to the topic
    /// @param message Message to publish
    void M3Publisher::PublishMessage(sensor_msgs::msg::PointCloud2 message){
        publisher_->publish(message);
    }
    void M3Publisher::CreateListener(std::string addr, u_int16_t port){
        M3Listener listener (addr, port, data_vector_, mutex_, new_header_);
        listener.create_socket();
        listener.connect_to_sonar();
        listener.run_listener();
    } 
    // M3Publisher::~M3Publisher(){
        

    // }
}

int main(int argc, char *argv[]) {
    // const char *ip_addr = argv[1];
    // std::stringstream strValue;
    // strValue << argv[2];
    // uint16_t port;
    // strValue >> port;

    // m3::M3PclPublisher publisher (argv[3]); // object that publishes the data extracted from the API
    // rclcpp::init(argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<m3::M3Publisher>());
    rclcpp::shutdown();
    
    return 0;
}
