#include <m3_driver/imb_parser.hpp>

namespace m3 {
namespace imb {

PacketHeader::PacketHeader(const uint8_t* byteArray) {
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

PacketFooter::PacketFooter(const uint8_t* byteArray) {
    std::memcpy(this, byteArray, sizeof(PacketFooter));
}

ImbPacketStructure::ImbPacketStructure(const uint8_t* byteArray)
    : packet_header(byteArray)
    , data_header(byteArray + sizeof(PacketHeader))
    , dataBody(byteArray + sizeof(PacketHeader) + sizeof(DataHeader),
               data_header.nNumBeams, data_header.nNumImageSample, packet_header.dataType)
    , packet_footer(byteArray + sizeof(PacketHeader) + sizeof(DataHeader) + dataBody.size)
{
    Validate();
    if(!validated){
        throw std::runtime_error("Packet validation failed");
    }
    GeneratePointCloud();
}

void ImbPacketStructure::Validate(){
    validated = (packet_header.packet_body_size == packet_footer.packet_body_size);
}

void ImbPacketStructure::GeneratePointCloud(){
    pcl::PointCloud<pcl::PointXYZI> cloud_local;
    cloud_local.height = 1;
    cloud_local.width = data_header.nNumBeams * data_header.nNumImageSample;
    cloud_local.is_dense = false;
    cloud_local.points.resize(cloud_local.height * cloud_local.width);

    for(size_t i = 0; i < data_header.nNumBeams; i++){
        float beam_angle = data_header.fBeamList[i];
        for(size_t j = 0; j < data_header.nNumImageSample; j++){
            pcl::PointXYZI point;
            float dist_to_point = ((data_header.fSWST - data_header.fTXWST)
                                    + data_header.fImageSampleInterval * j)
                                  * data_header.fSoundSpeed / 2;
            point.x = dist_to_point * std::cos(beam_angle * M_PI / 180.0);
            point.y = dist_to_point * std::sin(beam_angle * M_PI / 180.0);
            point.z = 0;
            point.intensity = std::max(std::abs(dataBody.complexData(i, j)), 0.0f);
            cloud_local.points[i * data_header.nNumImageSample + j] = point;
        }
    }
    cloud = cloud_local;
}

} // namespace imb
} // namespace m3
