#include <imb/ImbFormat.hpp>


namespace m3 {
namespace imb {

PacketHeader::PacketHeader(const uint8_t* byteArray)
{
    std::memcpy(this, byteArray, sizeof(PacketHeader));
}

DataHeader::DataHeader(const uint8_t* byte_array){
    std::memcpy(this, byte_array, sizeof(DataHeader));
}




DataBody::DataBody(const uint8_t* byteArray, uint16_t nNumBeams, uint16_t nNumSamples, DataType dataType) 
    : dataType(dataType) 
{
  
    switch(dataType) {
    case DataType::FloatComplex:
        size = nNumBeams * nNumSamples * sizeof(std::complex<float>);

        complexData = Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic>(nNumBeams, nNumSamples);
        for (size_t i = 0; i < nNumBeams; i++) {
            for (size_t j = 0; j < nNumSamples; j++) {
                std::complex<float> complex;
                std::memcpy(&complex, byteArray + (i * nNumSamples + j) * sizeof(std::complex<float>), sizeof(std::complex<float>));
                complexData(i, j) = complex;
            }
        }
        break;
    case DataType::IntegerMagnitude:
        size = nNumBeams * nNumSamples * sizeof(uint8_t);

        magnitudeData = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>(nNumBeams, nNumSamples);
        for (size_t i = 0; i < nNumBeams; ++i) {
            for (size_t j = 0; j < nNumSamples; ++j) {
                magnitudeData(i, j) = byteArray[i * nNumSamples + j];
            }
        }
        break;
    default:
        throw std::runtime_error("Invalid data type");
    }
}

PacketFooter::PacketFooter(const uint8_t* byteArray)
{
    std::memcpy(this, byteArray, sizeof(PacketFooter));
}

ImbPacketStructure::ImbPacketStructure(const uint8_t* byteArray)
    : packet_header(byteArray)
    , data_header  (byteArray + sizeof(PacketHeader))
    , dataBody    (byteArray + sizeof(PacketHeader) + sizeof(DataHeader), data_header.nNumBeams, data_header.nNumImageSample, packet_header.dataType)
    , packet_footer(byteArray + sizeof(PacketHeader) + sizeof(DataHeader) + dataBody.size)
{
    this->Validate();
    if(!this->validated){
        return;
    }
    this->GeneratePointCloud();

}

void ImbPacketStructure::Validate() const{
    if(this->packet_header.packet_body_size != packet_footer.packet_body_size){
        this->validated = false;
    }
    return;
}

void ImbPacketStructure::GeneratePointCloud() const{
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
            point.x = dist_to_point * cos(beam_angle * M_PI / 180.0);
            point.y = dist_to_point * sin(beam_angle * M_PI / 180.0);
            point.z = 0;
            point.intensity = std::max(abs(dataBody.complexData(i, j)), 0.0f);
            cloud.points[i * data_header.nNumImageSample + j] = point;
        }
    }
    // Convert the pointcloud to a ROS message
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.frame_id = "m3_sonar";
    message.header.stamp = rclcpp::Time(this->data_header.dwTimeSec, this->data_header.dwTimeMillisec); // Timestamp from sonar data
    this->message = message;
    return;

}
} // namespace imb
} // namespace m3