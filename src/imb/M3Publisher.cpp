
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
#include <pcl_conversions/pcl_conversions.h>
#include <imb/M3Listener.hpp>

using namespace std::chrono_literals;
namespace m3{
    M3Publisher::M3Publisher() : Node("M3_API_Publisher"){
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("m3/points", 10);
        data_vector_ = std::vector<uint8_t>();
        new_packet_ = false;
        std::thread(&M3Publisher::CreateListener, this, "10.0.0.153", 20001U).detach();
        timer_ = this->create_wall_timer(500ms, std::bind(&M3Publisher::ProcessData, this));
    }
    void M3Publisher::ProcessData() {
        try{
            // std::cout << new_packet_ << std::endl;
            if(new_packet_){
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
                // for(auto beam : data_header.fBeamList){
                //     if (beam != 0.0){
                //         std::cout << beam << std::endl;
                //     }
                // }
                //Inintialize the pointcloud
                // pcl::PointCloud<pcl::PointXYZI> cloud;
                // cloud.height = 1;
                // cloud.width = data_header.nNumBeams * data_header.nNumImageSample;
                // cloud.is_dense = false;
                // cloud.points.resize(cloud.height * cloud.width);

                // for(size_t i = 0; i < data_header.nNumBeams; i++){
                //     for(size_t j = 0; j < data_header.nNumImageSample; j++){
                //         pcl::PointXYZI point;
                //         point.x = data_body.complexData[i][j].real();
                //         point.y = data_body.complexData[i][j].imag();
                //         point.z = 0;
                //         point.intensity = 0;
                //         cloud.points[i * data_header.nNumImageSample + j] = point;
                //     }
                // }
                

                // TODO - find a way to create the pointcloud2 message directly as sensor_msg
                // std::vector<sensor_msgs::msg::PointField> points = std::vector<sensor_msgs::msg::PointField>();
                // sensor_msgs::msg::PointCloud2 message;
                // message.fields = points;

                new_packet_ = false; // Ready to receive new packet
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
        M3Listener listener (addr, port, data_vector_, mutex_, new_packet_);
        listener.create_socket();
        listener.connect_to_sonar();
        listener.run_listener();
    } 
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
