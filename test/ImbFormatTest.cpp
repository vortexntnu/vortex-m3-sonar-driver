#include <gtest/gtest.h>
#include <stdio.h>
#include <limits>
#include <inttypes.h>
#include <imb/ImbFormat.hpp>


TEST(ImbFormatSize, PacketHeader) {
    using namespace m3::imb;
    size_t size = sizeof(PacketHeader::synchronizationWord) 
                + sizeof(PacketHeader::dataType) 
                + sizeof(PacketHeader::reservedField) 
                + sizeof(PacketHeader::reservedBytes) 
                + sizeof(PacketHeader::packet_body_size);

    ASSERT_EQ(sizeof(PacketHeader), size);
}

TEST(ImbFormatSize, DataHeader) {
    size_t size = sizeof(m3::imb::DataHeader::dwBeamformed_Data_Version)
                + sizeof(m3::imb::DataHeader::dwSonarID)
                + sizeof(m3::imb::DataHeader::dwSonarInfo)
                + sizeof(m3::imb::DataHeader::dwTimeSec)
                + sizeof(m3::imb::DataHeader::dwTimeMillisec)
                + sizeof(m3::imb::DataHeader::fSoundSpeed)
                + sizeof(m3::imb::DataHeader::nNumImageSample)
                + sizeof(m3::imb::DataHeader::fNearRangeMeter)
                + sizeof(m3::imb::DataHeader::fFarRangeMeter)
                + sizeof(m3::imb::DataHeader::fSWST)
                + sizeof(m3::imb::DataHeader::fSWL)
                + sizeof(m3::imb::DataHeader::nNumBeams)
                + sizeof(m3::imb::DataHeader::wProcessingType)
                + sizeof(m3::imb::DataHeader::fBeamList)
                + sizeof(m3::imb::DataHeader::fImageSampleInterval)
                + sizeof(m3::imb::DataHeader::nImageDestination)
                + sizeof(m3::imb::DataHeader::bTXPulseType)
                + sizeof(m3::imb::DataHeader::bAzimuthProcessing)
                + sizeof(m3::imb::DataHeader::dwModeID)
                + sizeof(m3::imb::DataHeader::nNumHybridPRIs)
                + sizeof(m3::imb::DataHeader::nHybridIndex)
                + sizeof(m3::imb::DataHeader::nPhaseSeqLength)
                + sizeof(m3::imb::DataHeader::nPhaseSeqIndex)
                + sizeof(m3::imb::DataHeader::nNumImages)
                + sizeof(m3::imb::DataHeader::nSubImageIndex)
                + sizeof(m3::imb::DataHeader::dwFrequency)
                + sizeof(m3::imb::DataHeader::dwPulseLength)
                + sizeof(m3::imb::DataHeader::dwPingNumber)
                + sizeof(m3::imb::DataHeader::fRxFilterBW)
                + sizeof(m3::imb::DataHeader::fRxNominalResolution)
                + sizeof(m3::imb::DataHeader::fPulseRepFreq)
                + sizeof(m3::imb::DataHeader::strAppName)
                + sizeof(m3::imb::DataHeader::strTXPulseName)
                + sizeof(m3::imb::DataHeader::sTVGParameters)
                + sizeof(m3::imb::DataHeader::fCompassHeading)
                + sizeof(m3::imb::DataHeader::fMagneticVariation)
                + sizeof(m3::imb::DataHeader::fPitch)
                + sizeof(m3::imb::DataHeader::fRoll)
                + sizeof(m3::imb::DataHeader::fDepth)
                + sizeof(m3::imb::DataHeader::fTemperature)
                + sizeof(m3::imb::DataHeader::sOffsets)
                + sizeof(m3::imb::DataHeader::dbLatitude)
                + sizeof(m3::imb::DataHeader::dbLongitude)
                + sizeof(m3::imb::DataHeader::fTXWST)
                + sizeof(m3::imb::DataHeader::bHeadSensorVersion)
                + sizeof(m3::imb::DataHeader::HeadHWStatus)
                + sizeof(m3::imb::DataHeader::bSoundSpeedSource)
                + sizeof(m3::imb::DataHeader::bTimeSyncMode)
                + sizeof(m3::imb::DataHeader::fInternalSensorHeading)
                + sizeof(m3::imb::DataHeader::fInternalSensorPitch)
                + sizeof(m3::imb::DataHeader::fInternalSensorRoll)
                + sizeof(m3::imb::DataHeader::sAxesRotatorOffsets)
                + sizeof(m3::imb::DataHeader::nStartElement)
                + sizeof(m3::imb::DataHeader::nEndElement)
                + sizeof(m3::imb::DataHeader::strCustomText)
                + sizeof(m3::imb::DataHeader::strROVText)
                + sizeof(m3::imb::DataHeader::fLocalTimeOffset)
                + sizeof(m3::imb::DataHeader::fVesselSOG)
                + sizeof(m3::imb::DataHeader::fHeave)
                + sizeof(m3::imb::DataHeader::fPRIMinRange)
                + sizeof(m3::imb::DataHeader::fPRIMaxRange)
                + sizeof(m3::imb::DataHeader::fAltitude)
                + sizeof(m3::imb::DataHeader::fHeightGeoidAbvEllipsoid)
                + sizeof(m3::imb::DataHeader::sRealTimeSoundSpeed)
                + sizeof(m3::imb::DataHeader::sProfilingDepthTracking)
                + sizeof(m3::imb::DataHeader::sGPSQualityParas)
                + sizeof(m3::imb::DataHeader::b3DScan)
                + sizeof(m3::imb::DataHeader::b3DScanAxis)
                + sizeof(m3::imb::DataHeader::bSonarClassID)
                + sizeof(m3::imb::DataHeader::byTX_Enabled)
                + sizeof(m3::imb::DataHeader::bReserved)
                + sizeof(m3::imb::DataHeader::fROV_Altitude)
                + sizeof(m3::imb::DataHeader::fConductivity)
                + sizeof(m3::imb::DataHeader::reserved);

    ASSERT_EQ(sizeof(m3::imb::DataHeader), size);
}

TEST(ImbFormatSize, PacketFooter) {
    using namespace m3::imb;
    size_t size = sizeof(PacketFooter::packet_body_size) 
                + sizeof(PacketFooter::reserved_bytes);

    ASSERT_EQ(sizeof(PacketFooter), size);
}

TEST(ImbFormatParsing, TestPacket){
    using namespace m3::imb;

    std::vector<uint8_t> data;
    std::ifstream file("../../install/vortex-m3-sonar-driver/data/data/test_packet.txt", std::ios::in | std::ios::binary);
    if (file.is_open()) {
        // Get the size of the file
        file.seekg(0, std::ios::end);
        std::streampos fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        // Resize the vector to hold the file contents
        data.resize(static_cast<size_t>(fileSize));

        // Read the file content into the vector
        file.read(reinterpret_cast<char*>(data.data()), fileSize);

        // Close the file
        file.close();

        ImbPacketStructure packet(data.data());

        //Check synchonization words
        ASSERT_EQ(packet.packet_header.synchronizationWord[0], 0x8000);
        ASSERT_EQ(packet.packet_header.synchronizationWord[1], 0x8000);
        ASSERT_EQ(packet.packet_header.synchronizationWord[2], 0x8000);
        ASSERT_EQ(packet.packet_header.synchronizationWord[3], 0x8000);

        //Assert data size consistency
        ASSERT_EQ(packet.packet_header.packet_body_size, packet.packet_footer.packet_body_size);
        std::cout << std::endl;
    } else {
        FAIL() << "Error opening file" << std::endl;
    }
}




