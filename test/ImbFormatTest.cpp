#include <gtest/gtest.h>

#include <imb/ImbFormat.hpp>
#include "../src/netUtils.hpp"

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
    66b06326
    5a503000
    000a0b74
    45c05000

    0cdcc4c3
    e0000204
    12e53933
    9c72d663
    c800
    0000
    00d2 06ec20d6 06ac20da 066c20de 062c20d2 05fc20d6 05bc20da 057c20de 053c20d2050c20d604cc20da048c20de044c20d2041c20d603dc20da039c20de035c20d2032c20d602ec20da02ac20de026c20d2023c20d601fc20da01bc20de017c20d2014c20d6010c20da00cc20de008c20d2005c20d6001c21a40fbc11ac0f3c11a40ecc11ac0e4c11a40ddc11ac0d5c11a40cec11ac0c6c11a40bfc11ac0b7c11a40b0c11ac0a8c11a40a1c11ac099c11a4092c11ac08ac11a4083c1348077c1348068c1348059c134804ac134803bc134802cc100801dc100800ec10000ffc00000e1c00000c3c00000a5c0000087c0000052c0000016c00000b4bf0000f0be0000f03e0000b43f0000164000005240000087400000a5400000c3400000e1400000ff4000800e4100801d4134802c4134803b4134804a413480594134806841348077411a4083411ac08a411a4092411ac099411a40a1411ac0a8411a40b0411ac0b7411a40bf411ac0c6411a40ce411ac0d5411a40dd411ac0e4411a40ec411ac0f3411a40fb410d6001420d2005420de008420da00c420d6010420d2014420de017420da01b420d601f420d2023420de026420da02a420d602e420d2032420de035420da039420d603d420d2041420de044420da048420d604c420d2050420de053420da057420d605b420d205f420de062420da066420d606a420d206e4200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
    0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
)";

std::string REAL_DATA = R"(
    0080 0080 0080 0080
    0210
    0000
    00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
    40911500

    0e000000
    00000000
    16000819 00000000 00000000 00000000 00000000 00000000 00000000 00000000
    66b06326
    5a503000
    000a0b74
    45c05000
    0cdc
    c4c3e000
    020412e5
    39339c72
    d663
    c800
    0000
    00d2 06ec20d6 06ac20da 066c20de 062c20d2 05fc20d6 05bc20da 057c20de 053c20d2050c20d604cc20da048c20de044c20d2041c20d603dc20da039c20de035c20d2032c20d602ec20da02ac20de026c20d2023c20d601fc20da01bc20de017c20d2014c20d6010c20da00cc20de008c20d2005c20d6001c21a40fbc11ac0f3c11a40ecc11ac0e4c11a40ddc11ac0d5c11a40cec11ac0c6c11a40bfc11ac0b7c11a40b0c11ac0a8c11a40a1c11ac099c11a4092c11ac08ac11a4083c1348077c1348068c1348059c134804ac134803bc134802cc100801dc100800ec10000ffc00000e1c00000c3c00000a5c0000087c0000052c0000016c00000b4bf0000f0be0000f03e0000b43f0000164000005240000087400000a5400000c3400000e1400000ff4000800e4100801d4134802c4134803b4134804a413480594134806841348077411a4083411ac08a411a4092411ac099411a40a1411ac0a8411a40b0411ac0b7411a40bf411ac0c6411a40ce411ac0d5411a40dd411ac0e4411a40ec411ac0f3411a40fb410d6001420d2005420de008420da00c420d6010420d2014420de017420da01b420d601f420d2023420de026420da02a420d602e420d2032420de035420da039420d603d420d2041420de044420da048420d604c420d2050420de053420da057420d605b420d205f420de062420da066420d606a420d206e4200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
    00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
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
    using namespace m3::imb;
    size_t size = sizeof(DataHeader::dwBeamformed_Data_Version)
                + sizeof(DataHeader::dwSonarID)
                + sizeof(DataHeader::dwSonarInfo)
                + sizeof(DataHeader::dwTimeSec)
                + sizeof(DataHeader::dwTimeMillisec)
                + sizeof(DataHeader::fSoundSpeed)
                + sizeof(DataHeader::nNumImageSample)
                + sizeof(DataHeader::fNearRangeMeter)
                + sizeof(DataHeader::fFarRangeMeter)
                + sizeof(DataHeader::fSWST)
                + sizeof(DataHeader::fSWL)
                + sizeof(DataHeader::nNumBeams)
                + sizeof(DataHeader::wProcessingType)
                + sizeof(DataHeader::fBeamList)
                + sizeof(DataHeader::fImageSampleInterval)
                + sizeof(DataHeader::nImageDestination)
                + sizeof(DataHeader::bTXPulseType)
                + sizeof(DataHeader::bAzimuthProcessing)
                + sizeof(DataHeader::dwModeID)
                + sizeof(DataHeader::nNumHybridPRIs)
                + sizeof(DataHeader::nHybridIndex)
                + sizeof(DataHeader::nPhaseSeqLength)
                + sizeof(DataHeader::nPhaseSeqIndex)
                + sizeof(DataHeader::nNumImages)
                + sizeof(DataHeader::nSubImageIndex)
                + sizeof(DataHeader::dwFrequency)
                + sizeof(DataHeader::dwPulseLength)
                + sizeof(DataHeader::dwPingNumber)
                + sizeof(DataHeader::fRxFilterBW)
                + sizeof(DataHeader::fRxNominalResolution)
                + sizeof(DataHeader::fPulseRepFreq)
                + sizeof(DataHeader::strAppName)
                + sizeof(DataHeader::strTXPulseName);
                + sizeof(DataHeader::sTVGParameters)
                + sizeof(DataHeader::fCompassHeading)
                + sizeof(DataHeader::fMagneticVariation)
                + sizeof(DataHeader::fPitch)
                + sizeof(DataHeader::fRoll)
                + sizeof(DataHeader::fDepth)
                + sizeof(DataHeader::fTemperature)
                + sizeof(DataHeader::sOffsets)
                + sizeof(DataHeader::dbLatitude)
                + sizeof(DataHeader::dbLongitude)
                + sizeof(DataHeader::fTXWST)
                + sizeof(DataHeader::bHeadSensorVersion)
                + sizeof(DataHeader::HeadHWStatus)
                + sizeof(DataHeader::bSoundSpeedSource)
                + sizeof(DataHeader::bTimeSyncMode)
                + sizeof(DataHeader::fInternalSensorHeading)
                + sizeof(DataHeader::fInternalSensorPitch)
                + sizeof(DataHeader::fInternalSensorRoll)
                + sizeof(DataHeader::sAxesRotatorOffsets)
                + sizeof(DataHeader::nStartElement)
                + sizeof(DataHeader::strCustomText)
                + sizeof(DataHeader::strROVText)
                + sizeof(DataHeader::fLocalTimeOffset)
                + sizeof(DataHeader::fVesselSOG)
                + sizeof(DataHeader::fHeave)
                + sizeof(DataHeader::fPRIMinRange)
                + sizeof(DataHeader::fPRIMaxRange)
                + sizeof(DataHeader::fAltitude)
                + sizeof(DataHeader::fHeightGeoidAbvEllipsoid)
                + sizeof(DataHeader::sRealTimeSoundSpeed)
                + sizeof(DataHeader::sProfilingDepthTracking)
                + sizeof(DataHeader::sGPSQualityParas)
                + sizeof(DataHeader::b3DScan)
                + sizeof(DataHeader::b3DScanAxis)
                + sizeof(DataHeader::bSonarClassID)
                + sizeof(DataHeader::byTX_Enabled)
                + sizeof(DataHeader::bReserved)
                + sizeof(DataHeader::fROV_Altitude)
                + sizeof(DataHeader::fConductivity)
                + sizeof(DataHeader::Reserved);
    size += sizeof(DataHeader::Reserved);

    ASSERT_EQ(sizeof(DataHeader), size);
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
    addDataString(DATA_HEADER_DATA);
    ASSERT_GE(byte_array_size_, sizeof(m3::imb::DataHeader));
    m3::imb::DataHeader dh(byte_array_);

    std::cout << dh.strAppName << '\n';

    ASSERT_EQ(dh.dwBeamformed_Data_Version, 14U);
    ASSERT_EQ(dh.dwSonarID                , 0U);
    ASSERT_EQ(dh.dwSonarInfo[0]           , 419954710U);
    ASSERT_EQ(dh.dwTimeSec                , 644067430U);
    ASSERT_EQ(dh.dwTimeMillisec           , 3166298U);
    // ASSERT_NEAR(dh.fSoundSpeed            , 1500.0, 1e-6);
    ASSERT_EQ(dh.nNumImageSample          , 5292101U);
    // ASSERT_NEAR(dh.fNearRangeMeter        , 0.0, 1e-6);
    // ASSERT_NEAR(dh.fFarRangeMeter         , 100.0, 1e-6);
    // ASSERT_NEAR(dh.fSWST                  , 0.0, 1e-6);
    // ASSERT_NEAR(dh.fSWL                   , 0.0, 1e-6);
    ASSERT_EQ(dh.nNumBeams                , 200U);
    ASSERT_EQ(dh.wProcessingType          , 0U);
    for (size_t i = 0; i < 1024; i++) {
        // ASSERT_NEAR(dh.fBeamList[i], 0.0, 360);
    }
    // ASSERT_NEAR(dh.fImageSampleInterval   , 0.0, 1e-6);
    ASSERT_EQ(dh.nImageDestination        , 12336U);
    ASSERT_EQ(dh.bTXPulseType             , '0'); // TODO: Figure out why this is a char '0' instead of a uint8_t 0
    ASSERT_EQ(dh.bAzimuthProcessing       , '0');
    ASSERT_EQ(dh.dwModeID                 , 808464432U);
    ASSERT_EQ(dh.nNumHybridPRIs           , 0U);
    ASSERT_EQ(dh.nHybridIndex             , 0U);
    ASSERT_EQ(dh.nPhaseSeqLength          , 0U);
    ASSERT_EQ(dh.nPhaseSeqIndex           , 0U);
    ASSERT_EQ(dh.nNumImages               , 0U);
    ASSERT_EQ(dh.nSubImageIndex           , 0U);
    ASSERT_EQ(dh.dwFrequency              , 0U);
    ASSERT_EQ(dh.dwPulseLength            , 0U);
    ASSERT_EQ(dh.dwPingNumber             , 0U);
    // ASSERT_NEAR(dh.fRxFilterBW, 0.0, 1e-6);
    // ASSERT_NEAR(dh.fRxNominalResolution, 0.0, 1e-6);
    // ASSERT_NEAR(dh.fPulseRepFreq, 0.0, 1e-6);
    ASSERT_STREQ(dh.strAppName, "");
    ASSERT_STREQ(dh.strTXPulseName, "");
    
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
    uint32_t hexValue = 0x000a0b74;
    hexValue = ntohl(hexValue);

    int sign = ((hexValue >> 31) == 0) ? 1 : -1;
    int exponent = ((hexValue >> 23) & 0xff);
    int mantissa = (exponent == 0) ? (hexValue & 0x7fffff) << 1 : (hexValue & 0x7fffff) | 0x800000;

    float f = sign * mantissa * std::pow(2, exponent - 150);  // The 150 is 127 (bias) + 23 (mantissa bits)

    ASSERT_NEAR(f, 1500, 1e-6);

}