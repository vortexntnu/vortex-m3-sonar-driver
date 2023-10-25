#include <gtest/gtest.h>
#include <arpa/inet.h>

#include "../src/utils/netUtils.hpp"

TEST(netUtils, test_is_little_endian)
{
    bool result = m3::utils::is_little_endian();
    EXPECT_EQ(result, true);
}

TEST(netUtils, test_ntohs)
{
    uint16_t test = 0x1234;
    uint16_t result = ntohs(test);
    EXPECT_EQ(result, 0x3412);
}

TEST(netUtils, test_ntohl)
{
    uint32_t test = 0x12345678;
    uint32_t result = ntohl(test);
    EXPECT_EQ(result, (uint32_t)0x78563412);
}

TEST(netUtils, test_ntohd)
{
    uint64_t test = 0x1234567890ABCDEF;
    uint64_t result = m3::utils::ntohd(test);
    EXPECT_EQ(result, 0xEFCDAB9078563412);
}

TEST(netUtils, HexStringToByteArray_8_bytes)
{
    std::string hexString = "1234567890ABCDEF";
    unsigned char* byteArray = new unsigned char[8];
    m3::utils::hexStringToByteArray(hexString, byteArray, 8);
    EXPECT_EQ(byteArray[0], 0x12);
    EXPECT_EQ(byteArray[1], 0x34);
    EXPECT_EQ(byteArray[2], 0x56);
    EXPECT_EQ(byteArray[3], 0x78);
    EXPECT_EQ(byteArray[4], 0x90);
    EXPECT_EQ(byteArray[5], 0xAB);
    EXPECT_EQ(byteArray[6], 0xCD);
    EXPECT_EQ(byteArray[7], 0xEF);
    delete[] byteArray;
}

TEST(netUtils, HexStringToByteArray_string_with_spaces)
{
    std::string hexString = "12 34 56 78 90 AB CD EF";
    unsigned char* byteArray = new unsigned char[8];
    m3::utils::hexStringToByteArray(hexString, byteArray, 8);
    EXPECT_EQ(byteArray[0], 0x12);
    EXPECT_EQ(byteArray[1], 0x34);
    EXPECT_EQ(byteArray[2], 0x56);
    EXPECT_EQ(byteArray[3], 0x78);
    EXPECT_EQ(byteArray[4], 0x90);
    EXPECT_EQ(byteArray[5], 0xAB);
    EXPECT_EQ(byteArray[6], 0xCD);
    EXPECT_EQ(byteArray[7], 0xEF);
    delete[] byteArray;
}

TEST(netUtils, HexStringToByteArray_sync_word)
{
    std::string hexString = "0080 0080 0080 0080";
    unsigned char* byteArray = new unsigned char[8];
    m3::utils::hexStringToByteArray(hexString, byteArray, 8);
    EXPECT_EQ(byteArray[0], 0x00);
    EXPECT_EQ(byteArray[1], 0x80);
    EXPECT_EQ(byteArray[2], 0x00);
    EXPECT_EQ(byteArray[3], 0x80);
    EXPECT_EQ(byteArray[4], 0x00);
    EXPECT_EQ(byteArray[5], 0x80);
    EXPECT_EQ(byteArray[6], 0x00);
    EXPECT_EQ(byteArray[7], 0x80);
    delete[] byteArray;
}