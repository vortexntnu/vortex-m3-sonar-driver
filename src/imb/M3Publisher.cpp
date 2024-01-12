#include <imb/M3Publisher.hpp>
using namespace std::chrono_literals;
namespace m3{
    M3Publisher::M3Publisher() : Node("M3Publisher"){
        this->declare_parameter("ip_addr", rclcpp::PARAMETER_STRING);
        this->declare_parameter("port", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("topic", rclcpp::PARAMETER_STRING);
        this->declare_parameter("callback_time", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("reliable_qos", rclcpp::PARAMETER_BOOL);

        
        if(this->get_parameter("reliable_qos").as_bool()){
            // Define the quality of service profile for publisher to match the other publishers and subscribers
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("topic").as_string().c_str(), qos);
        }
        else{
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->get_parameter("topic").as_string().c_str(), 10);
        }

        data_vector_ = std::vector<uint8_t>();
        packet_ready_ = false;
        std::thread(&M3Publisher::CreateListener, this, this->get_parameter("ip_addr").as_string(), this->get_parameter("port").as_int()).detach();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("callback_time").as_int()), std::bind(&M3Publisher::ProcessData, this)); // How often to sample data from listener
    }
    void M3Publisher::ProcessData() {
        try{
            if(!packet_ready_){
                return;
            }
            std::unique_lock<std::mutex> lock(mutex_); // Thread-safe handling
            const uint8_t* start = data_vector_.data(); // Grabs the pointer to the first element in the vector
            imb::ImbPacketStructure packet(start); // Parse the packet to an object
    
            PublishMessage(packet.message);
        
            packet_ready_ = false; // Ready to receive new packet
            lock.unlock();
        }
        catch(const std::exception& e){
            std::cerr << e.what() << '\n';
        }
       
    }
    void M3Publisher::PublishMessage(sensor_msgs::msg::PointCloud2 message){
        publisher_->publish(message);
    }
    void M3Publisher::CreateListener(std::string addr, uint16_t port){
        M3Listener listener (addr, port, data_vector_, mutex_, packet_ready_);
        listener.create_socket();
        listener.connect_to_sonar();
        listener.run_listener();
    } 
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<m3::M3Publisher>());
    rclcpp::shutdown();
    return 0;
}
