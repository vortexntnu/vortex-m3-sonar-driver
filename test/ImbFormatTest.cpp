#include <gtest/gtest.h>
#include <stdio.h>
#include <limits>
#include <inttypes.h>
#include <imb/ImbFormat.hpp>
#include "../src/utils/netUtils.hpp"

std::string PACKET_HEADER_DATA = R"(
    0080 0080 0080 0080
    0210
    0000
    00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
    40911500
)";
std::string DATA_HEADER_DATA = R"(
    0e000000
    00000000
    16000819 00000000 00000000 00000000 00000000 00000000 00000000 00000000
    37b83265
    31030000
    00a0b744
    5c050000
    cdcc4c3e
    00002041
    2e539339
    c72d663c
)";

std::string TEST_REAL_DATA = R"(
  00 80 00 80 00 80 00 80 02 10
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 40 91 15 00 0e 00
  00 00 00 00 00 00 16 00 08 19 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 37 b8 32 65 31 03 00 00 00 a0
  b7 44 5c 05 00 00 cd cc 4c 3e 00 00 20 41 2e 53
  93 39 c7 2d 66 3c 80 00 00 00 0d 20 6e c2 0d 60
  6a c2 0d a0 66 c2 0d e0 62 c2 0d 20 5f c2 0d 60
  5b c2 0d a0 57 c2 0d e0 53 c2 0d 20 50 c2 0d 60
  4c c2 0d a0 48 c2 0d e0 44 c2 0d 20 41 c2 0d 60
  3d c2 0d a0 39 c2 0d e0 35 c2 0d 20 32 c2 0d 60
  2e c2 0d a0 2a c2 0d e0 26 c2 0d 20 23 c2 0d 60
  1f c2 0d a0 1b c2 0d e0 17 c2 0d 20 14 c2 0d 60
  10 c2 0d a0 0c c2 0d e0 08 c2 0d 20 05 c2 0d 60
  01 c2 1a 40 fb c1 1a c0 f3 c1 1a 40 ec c1 1a c0
  e4 c1 1a 40 dd c1 1a c0 d5 c1 1a 40 ce c1 1a c0
  c6 c1 1a 40 bf c1 1a c0 b7 c1 1a 40 b0 c1 1a c0
  a8 c1 1a 40 a1 c1 1a c0 99 c1 1a 40 92 c1 1a c0
  8a c1 1a 40 83 c1 34 80 77 c1 34 80 68 c1 34 80
  59 c1 34 80 4a c1 34 80 3b c1 34 80 2c c1 00 80
  1d c1 00 80 0e c1 00 00 ff c0 00 00 e1 c0 00 00
  c3 c0 00 00 a5 c0 00 00 87 c0 00 00 52 c0 00 00
  16 c0 00 00 b4 bf 00 00 f0 be 00 00 f0 3e 00 00
  b4 3f 00 00 16 40 00 00 52 40 00 00 87 40 00 00
  a5 40 00 00 c3 40 00 00 e1 40 00 00 ff 40 00 80
  0e 41 00 80 1d 41 34 80 2c 41 34 80 3b 41 34 80
  4a 41 34 80 59 41 34 80 68 41 34 80 77 41 1a 40
  83 41 1a c0 8a 41 1a 40 92 41 1a c0 99 41 1a 40
  a1 41 1a c0 a8 41 1a 40 b0 41 1a c0 b7 41 1a 40
  bf 41 1a c0 c6 41 1a 40 ce 41 1a c0 d5 41 1a 40
  dd 41 1a c0 e4 41 1a 40 ec 41 1a c0 f3 41 1a 40
  fb 41 0d 60 01 42 0d 20 05 42 0d e0 08 42 0d a0
  0c 42 0d 60 10 42 0d 20 14 42 0d e0 17 42 0d a0
  1b 42 0d 60 1f 42 0d 20 23 42 0d e0 26 42 0d a0
  2a 42 0d 60 2e 42 0d 20 32 42 0d e0 35 42 0d a0
  39 42 0d 60 3d 42 0d 20 41 42 0d e0 44 42 0d a0
  48 42 0d 60 4c 42 0d 20 50 42 0d e0 53 42 0d a0
  57 42 0d 60 5b 42 0d 20 5f 42 0d e0 62 42 0d a0
  66 42 0d 60 6a 42 0d 20 6e 42 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  00 00 00 00 00 00 00 00 00 00
)";


TEST(ImbFormatSize, PacketHeader) {
    using namespace m3::imb;
    size_t size = sizeof(PacketHeader::synchronizationWord) 
                + sizeof(PacketHeader::dataType) 
                + sizeof(PacketHeader::reservedField) 
                + sizeof(PacketHeader::reservedBytes) 
                + sizeof(PacketHeader::packetBodySize);

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
                + sizeof(m3::imb::DataHeader::Reserved);

    ASSERT_EQ(sizeof(m3::imb::DataHeader), size);
}




class ImbFormatData : public ::testing::Test {
protected:
    unsigned char* byte_array_;    
    size_t byte_array_size_;
    void addDataString(std::string hexString)
    {
        byte_array_size_ = hexString.length() / 2;

        byte_array_ = new unsigned char[byte_array_size_];
        hexStringToByteArray(hexString, byte_array_, byte_array_size_);
    }

    ~ImbFormatData() override {
        delete[] byte_array_;
    }
};

TEST_F(ImbFormatData, PacketHeader_correct_data) {
    addDataString(PACKET_HEADER_DATA);
    ASSERT_GE(byte_array_size_, sizeof(m3::imb::PacketHeader));

    m3::imb::PacketHeader ph(byte_array_);

    ASSERT_EQ(ph.synchronizationWord[0], 0x8000);
    ASSERT_EQ(ph.synchronizationWord[1], 0x8000);
    ASSERT_EQ(ph.synchronizationWord[2], 0x8000);
    ASSERT_EQ(ph.synchronizationWord[3], 0x8000);
    ASSERT_EQ(ph.dataType, m3::imb::DataType::FloatComplex);
    ASSERT_EQ(ph.packetBodySize, 1413440U);
}

TEST_F(ImbFormatData, DataHeader_correct_data) {
    addDataString(TEST_REAL_DATA);
    //ASSERT_GE(byte_array_size_, sizeof(m3::imb::DataHeader));
    m3::imb::DataHeader dh(byte_array_ + sizeof(m3::imb::PacketHeader));

    std::cout << dh.strAppName << '\n';

    // clang-format off
    ASSERT_EQ(dh.dwBeamformed_Data_Version, 14U);
    ASSERT_EQ(dh.dwSonarID                , 0U);
    ASSERT_EQ(dh.dwSonarInfo[0]           , 419954710U);            // Idk what this is
    ASSERT_NEAR(dh.dwTimeSec              , 1700000000, 1e9);       // Close enough
    ASSERT_LT(dh.dwTimeMillisec           , 1000000U);              // Less than 1 second
    ASSERT_NEAR(dh.fSoundSpeed            , 1500, 1e3);             // +- 100 m/s
    ASSERT_EQ(dh.nNumImageSample          , 1372U);
    ASSERT_GT(dh.fNearRangeMeter          , 0.0);                   // Positive
    ASSERT_LE(dh.fNearRangeMeter          , 150.0);                 // 150 meters is the max range
    ASSERT_GT(dh.fFarRangeMeter           , 0.0);                   // Positive
    ASSERT_LE(dh.fFarRangeMeter           , 150.0);                 // 150 meters is the max range
    ASSERT_GT(dh.fFarRangeMeter           , dh.fNearRangeMeter);    // Far range is greater than near range
    ASSERT_GT(dh.fSWST                    , 0.0);                   // Positive
    ASSERT_GT(dh.fSWL                     , 0.0);
    ASSERT_LE(dh.fSWL                     , 1.0);                   // Sample width less than 1 second is reasonable
    ASSERT_EQ(dh.nNumBeams                , 128);
    ASSERT_EQ(dh.wProcessingType          , 0U);
    for (size_t i = 0; i < 1024; i++) {
        ASSERT_NEAR(dh.fBeamList[i], 0.0, 60.0);                    // Angle is between -60 and 60 degrees
    }
    ASSERT_NEAR(dh.fImageSampleInterval   , 0.0, 1e-6); // TODO: Tests from here and down are not complete
    ASSERT_EQ(dh.nImageDestination        , 0U);
    ASSERT_EQ(dh.bTXPulseType             , 0U); 
    ASSERT_EQ(dh.dwModeID                 , 0U);
    ASSERT_EQ(dh.nNumHybridPRIs           , 0U);
    ASSERT_EQ(dh.nHybridIndex             , 0U);
    ASSERT_EQ(dh.nPhaseSeqLength          , 0U);
    ASSERT_EQ(dh.nPhaseSeqIndex           , 0U);
    ASSERT_EQ(dh.nNumImages               , 0U);
    ASSERT_EQ(dh.nSubImageIndex           , 0U);
    ASSERT_EQ(dh.dwFrequency              , 0U);
    ASSERT_EQ(dh.dwPulseLength            , 0U);
    ASSERT_EQ(dh.dwPingNumber             , 0U);
    ASSERT_NEAR(dh.fRxFilterBW            , 0.0, 1e-6);
    ASSERT_NEAR(dh.fRxNominalResolution   , 0.0, 1e-6);
    ASSERT_NEAR(dh.fPulseRepFreq          , 0.0, 1e-6);
    ASSERT_STREQ(dh.strAppName            , "");
    ASSERT_STREQ(dh.strTXPulseName        , "");
    ASSERT_EQ(dh.sTVGParameters           , 0U);
    ASSERT_EQ(dh.fCompassHeading          , 0U);
    ASSERT_EQ(dh.fMagneticVariation       , 0U);
    ASSERT_EQ(dh.fPitch                   , 0U);
    ASSERT_EQ(dh.fRoll                    , 0U);
    ASSERT_EQ(dh.fDepth                   , 0U);
    ASSERT_EQ(dh.fTemperature             , 0U);
    ASSERT_EQ(dh.sOffsets                 , 0U);
    ASSERT_EQ(dh.dbLatitude               , 0U);
    ASSERT_EQ(dh.dbLongitude              , 0U);
    ASSERT_EQ(dh.fTXWST                   , 0U);
    ASSERT_EQ(dh.bHeadSensorVersion       , 0U);
    ASSERT_EQ(dh.HeadHWStatus             , 0U);
    ASSERT_EQ(dh.bSoundSpeedSource        , 0U);
    ASSERT_EQ(dh.bTimeSyncMode            , 0U);
    ASSERT_EQ(dh.fInternalSensorHeading   , 0U);
    ASSERT_EQ(dh.fInternalSensorPitch     , 0U);
    ASSERT_EQ(dh.fInternalSensorRoll      , 0U);
    ASSERT_EQ(dh.sAxesRotatorOffsets      , 0U);
    ASSERT_EQ(dh.nStartElement            , 0U);
    ASSERT_EQ(dh.nEndElement              , 0U);
    ASSERT_EQ(dh.strCustomText            , 0U);
    ASSERT_EQ(dh.strROVText               , 0U);
    ASSERT_EQ(dh.fLocalTimeOffset         , 0U);
    ASSERT_EQ(dh.fVesselSOG               , 0U);
    ASSERT_EQ(dh.fHeave                   , 0U);
    ASSERT_EQ(dh.fPRIMinRange             , 0U);
    ASSERT_EQ(dh.fPRIMaxRange             , 0U);
    ASSERT_EQ(dh.fAltitude                , 0U);
    ASSERT_EQ(dh.fHeightGeoidAbvEllipsoid , 0U);
    ASSERT_EQ(dh.sRealTimeSoundSpeed      , 0U);
    ASSERT_EQ(dh.sProfilingDepthTracking  , 0U);
    ASSERT_EQ(dh.sGPSQualityParas         , 0U);
    ASSERT_EQ(dh.b3DScan                  , 0U);
    ASSERT_EQ(dh.b3DScanAxis              , 0U);
    ASSERT_EQ(dh.bSonarClassID            , 0U);
    ASSERT_EQ(dh.byTX_Enabled             , 0U);
    ASSERT_EQ(dh.bReserved                , 0U);
    ASSERT_EQ(dh.fROV_Altitude            , 0U);
    ASSERT_EQ(dh.fConductivity            , 0U);
    ASSERT_EQ(dh.Reserved                 , 0U);
    // clang-format on
    
}

TEST_F(ImbFormatData, data_alignment_headers) {
    addDataString(DATA_HEADER_DATA);
    ASSERT_GE(byte_array_size_, sizeof(m3::imb::PacketHeader) + sizeof(m3::imb::DataHeader));

    m3::imb::PacketHeader ph(byte_array_);
    m3::imb::DataHeader dh(byte_array_ + sizeof(m3::imb::PacketHeader));

    ASSERT_EQ(ph.synchronizationWord[0], 0x8000);
    ASSERT_EQ(dh.dwBeamformed_Data_Version, 14U);

}

TEST(ConvertFloat, test1) {
    // Assuming the hex value is a 32-bit float (IEEE 754)
    uint32_t hexValue = 0x00a0b744; //740b0a00 -> 1946880512
    hexValue = ntohl(hexValue);
    //auto max_float = std::numeric_limits<float>::max();
    //printf("%" PRIx32 "\n", MAXFLOAT);
    //auto test = hexValue >> 31;
    int sign = ((hexValue >> 31) == 0) ? 1 : -1;
    int exponent = ((hexValue >> 23) & 0xff);
    int mantissa = (exponent == 0) ? (hexValue & 0x7fffff) << 1 : (hexValue & 0x7fffff) | 0x800000;

    float f = sign * mantissa * std::pow(2, exponent - 150);  // The 150 is 127 (bias) + 23 (mantissa bits)

    ASSERT_NEAR(f, 1469, 1e-6);

}