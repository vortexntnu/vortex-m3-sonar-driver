// #include <imb/M3Publisher.hpp>
#include <imb/M3Listener.hpp>
namespace m3{
    M3Publisher::M3Publisher() : Node("M3_Data_Publisher"){
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("m3/points", 10);
        // std::thread listener_thread (&M3Publisher::CreateListener, this, "10.0.0.203", 20001U);
    }
    void M3Publisher::ProcessData(m3::imb::DataBody data) {
        std::cout << "Processing data" << std::endl;
    }
    void M3Publisher::PublishMessage(sensor_msgs::msg::PointCloud2 message){
        publisher_->publish(message);
    }
    // void M3Publisher::CreateListener(std::string addr, u_int16_t port, M3Publisher& publisher){
    //     M3Listener listener (addr, port, publisher);
    //     listener.create_socket();
    //     listener.connect_to_sonar();
    //     listener.run_listener();
    // } 
}
