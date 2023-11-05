#include <imb/ImbFormat.hpp>
#include "../utils/netUtils.hpp"
#include <iostream>

using m3::utils::ntohd;
namespace m3 {
namespace imb {


static constexpr uint16_t SYNC_WORD = 0x8000; // Always 4x 0x8000 at the start of each packet




PacketHeader::PacketHeader(const uint8_t* byteArray)
{
    std::memcpy(this, byteArray, sizeof(PacketHeader));
    // byte alignment
//     for (size_t i = 0; i < 4; i++)
//     {
//         // synchronizationWord[i] = ntohs(synchronizationWord[i]);
//         if (synchronizationWord[i] != SYNC_WORD) { throw std::invalid_argument("Invalid synchronization word"); }
//     }
//     // dataType = (DataType)ntohs((uint16_t)dataType);
//     if (dataType != DataType::FloatComplex && dataType != DataType::IntegerMagnitude) { throw std::invalid_argument("Invalid data type"); }
//     // don't need to convert reservedField and reservedBytes
//     // packetBodySize = ntohl(packetBodySize);
}
// PacketHeader::PacketHeader(){

// }
DataHeader::DataHeader(const uint8_t* byte_array){
    std::memcpy(this, byte_array, sizeof(DataHeader));
}

// DataHeader::DataHeader(const uint8_t* byteArray)
// {
    
//     std::memcpy(this, byteArray, sizeof(DataHeader));

//     double a = 1500;
//     uint32_t bytes = htonl(a);
//     std::cout << bytes << '\n';
//     std::cout << ntohl(a) << '\n';
//     // byte alignment
//     // dwBeamformed_Data_Version = ntohl(dwBeamformed_Data_Version);
//     // dwSonarID = ntohl(dwSonarID);
//     // for (size_t i = 0; i < 8; i++)
//     // {
//         // dwSonarInfo[i] = ntohl(dwSonarInfo[i]);
//     // }
//     // dwTimeSec = ntohl(dwTimeSec);
//     // dwTimeMillisec = ntohl(dwTimeMillisec);
//     // fSoundSpeed = ntohl(fSoundSpeed);
//     // nNumImageSample = ntohl(nNumImageSample);
//     // fNearRangeMeter = ntohl(fNearRangeMeter);
//     // fFarRangeMeter = ntohl(fFarRangeMeter);
//     // fSWST = ntohl(fSWST);
//     // fSWL = ntohl(fSWL);
//     // nNumBeams = ntohs(nNumBeams);
//     // wProcessingType = ntohs(wProcessingType);
//     // for (size_t i = 0; i < 1024; i++)
//     // {
//     //     fBeamList[i] = ntohl(fBeamList[i]);
//     // }
//     // fImageSampleInterval = ntohl(fImageSampleInterval);
//     // nImageDestination = ntohs(nImageDestination);
//     // dwModeID = ntohl(dwModeID);
//     // nNumHybridPRIs = ntohs(nNumHybridPRIs);
//     // nHybridIndex = ntohs(nHybridIndex);
//     // nPhaseSeqLength = ntohs(nPhaseSeqLength);
//     // nPhaseSeqIndex = ntohs(nPhaseSeqIndex);
//     // nNumImages = ntohs(nNumImages);
//     // nSubImageIndex = ntohs(nSubImageIndex);
//     // dwFrequency = ntohl(dwFrequency);
//     // dwPulseLength = ntohl(dwPulseLength);
//     // dwPingNumber = ntohl(dwPingNumber);
//     // fRxFilterBW = ntohl(fRxFilterBW);
//     // fRxNominalResolution = ntohl(fRxNominalResolution);
//     // fPulseRepFreq = ntohl(fPulseRepFreq);
//     // sTVGParameters.factorA = ntohs(sTVGParameters.factorA);
//     // sTVGParameters.factorB = ntohs(sTVGParameters.factorB);
//     // sTVGParameters.factorC = ntohl(sTVGParameters.factorC);
//     // sTVGParameters.factorL = ntohl(sTVGParameters.factorL);
//     // dbLatitude = ntohd(dbLatitude);
//     // dbLongitude = ntohd(dbLongitude);
//     // fTXWST = ntohl(fTXWST);
//     // fInternalSensorHeading = ntohl(fInternalSensorHeading);
//     // fInternalSensorPitch = ntohl(fInternalSensorPitch);
//     // fInternalSensorRoll = ntohl(fInternalSensorRoll);
//     // for (size_t i = 0; i < 3; i++)
//     // {
//     //     sAxesRotatorOffsets[i].fOffsetA = ntohl(sAxesRotatorOffsets[i].fOffsetA);
//     //     sAxesRotatorOffsets[i].fOffsetB = ntohl(sAxesRotatorOffsets[i].fOffsetB);
//     //     sAxesRotatorOffsets[i].fOffsetR = ntohl(sAxesRotatorOffsets[i].fOffsetR);
//     //     sAxesRotatorOffsets[i].fAngle = ntohl(sAxesRotatorOffsets[i].fAngle);
//     // }
//     // nStartElement = ntohs(nStartElement);
//     // nEndElement = ntohs(nEndElement);
//     // fLocalTimeOffset = ntohl(fLocalTimeOffset);
//     // fVesselSOG = ntohl(fVesselSOG);
//     // fHeave = ntohl(fHeave);
//     // fPRIMinRange = ntohl(fPRIMinRange);
//     // fPRIMaxRange = ntohl(fPRIMaxRange);
//     // fAltitude = ntohl(fAltitude);
//     // fHeightGeoidAbvEllipsoid = ntohl(fHeightGeoidAbvEllipsoid);
//     // sRealTimeSoundSpeed.fRaw = ntohl(sRealTimeSoundSpeed.fRaw);
//     // sRealTimeSoundSpeed.fFiltered = ntohl(sRealTimeSoundSpeed.fFiltered);
//     // sRealTimeSoundSpeed.fApplied = ntohl(sRealTimeSoundSpeed.fApplied);
//     // sRealTimeSoundSpeed.fThreshold = ntohl(sRealTimeSoundSpeed.fThreshold);
//     // sProfilingDepthTracking.nRangeDeepLimitPercentage = ntohs(sProfilingDepthTracking.nRangeDeepLimitPercentage);
//     // sProfilingDepthTracking.nRangeShallowLimitPercentage = ntohs(sProfilingDepthTracking.nRangeShallowLimitPercentage);
//     // sProfilingDepthTracking.nMinPingRateInHz = ntohs(sProfilingDepthTracking.nMinPingRateInHz);
//     // sGPSQualityParas.wQualIndicator = ntohs(sGPSQualityParas.wQualIndicator);
//     // sGPSQualityParas.wNSat = ntohs(sGPSQualityParas.wNSat);
//     // sGPSQualityParas.fHorizDilution = ntohl(sGPSQualityParas.fHorizDilution);
//     // sGPSQualityParas.wPosFixQualityCm = ntohs(sGPSQualityParas.wPosFixQualityCm);
//     // b3DScan = ntohl(b3DScan);
//     // b3DScanAxis = ntohl(b3DScanAxis);
//     // fROV_Altitude = ntohl(fROV_Altitude);
//     // fConductivity = ntohl(fConductivity);
// }

DataBody::DataBody(const uint8_t* byteArray, uint16_t nNumBeams, uint16_t nNumSamples, DataType dataType) 
    : dataType(dataType) 
{
  
    switch(dataType) {
    case DataType::FloatComplex:
        size = nNumBeams * nNumSamples * sizeof(std::complex<float>);

        complexData = Eigen::Matrix<std::complex<float>, Eigen::Dynamic, Eigen::Dynamic>(nNumBeams, nNumSamples);
        for (size_t i = 0; i < nNumBeams; ++i) {
            for (size_t j = 0; j < nNumSamples; ++j) {
                uint32_t complexI, complexQ;
                std::memcpy(&complexI, byteArray + (i * nNumSamples + j) * sizeof(std::complex<float>), sizeof(float));
                std::memcpy(&complexQ, byteArray + (i * nNumSamples + j) * sizeof(std::complex<float>) + sizeof(float), sizeof(float));

                // Convert from network byte order to host byte order
                complexI = ntohl(complexI);
                complexQ = ntohl(complexQ);

                // Assign the values to the complexData matrix
                complexData(i, j) = std::complex<float>(*reinterpret_cast<float*>(&complexI), *reinterpret_cast<float*>(&complexQ));
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
    // byte alignment
    packetBodySize = ntohl(packetBodySize);
    // don't need to convert reservedBytes
}

ImbPacketStructure::ImbPacketStructure(const uint8_t* byteArray)
    : packetHeader(byteArray)
    , dataHeader  (byteArray + sizeof(PacketHeader))
    , dataBody    (byteArray + sizeof(PacketHeader) + sizeof(DataHeader), dataHeader.nNumBeams, dataHeader.nNumImageSample, packetHeader.dataType)
    , packetFooter(byteArray + sizeof(PacketHeader) + sizeof(DataHeader) + dataBody.size)
{
    validate();
}

void ImbPacketStructure::validate() const
{
    for (size_t i = 0; i < 4; ++i) {
        if (packetHeader.synchronizationWord[i] != SYNC_WORD) {
            throw std::invalid_argument("Invalid synchronization word");
        }
    }
    if (packetHeader.dataType != DataType::FloatComplex && packetHeader.dataType != DataType::IntegerMagnitude) {
        throw std::invalid_argument("Invalid data type");
    }
    if (packetHeader.packetBodySize != packetFooter.packetBodySize) {
        throw std::invalid_argument("Invalid packet body size");
    }
    if (packetHeader.packetBodySize != sizeof(dataHeader) + dataBody.size) {
        throw std::invalid_argument("Invalid packet body size");
    }
}
} // namespace imb
} // namespace m3