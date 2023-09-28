#include <gtest/gtest.h>
#include <XmlResponseParser.hpp>

TEST(XmlResponseParser, ParseGetStatusResponseWithValidXml) {
    std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2016-09-13 15:22:21</Time>
</Response>)";

    ResponseInfo result = parseResponse(response);

    EXPECT_EQ(result.operation, "Get_Status");
    EXPECT_EQ(result.status, "OK");
    EXPECT_EQ(result.message, "Pinging");
    EXPECT_EQ(result.version, "2.1.0");
    EXPECT_EQ(result.time, "2016-09-13 15:22:21");
}

TEST(XmlResponseParser, ParseGetStatusResponseWithInvalidXml) {
    std::string response = R"(Invalid XML)";

    ResponseInfo result = parseResponse(response);

    EXPECT_EQ(result.operation, "");
    EXPECT_EQ(result.status, "");
    EXPECT_EQ(result.message, "");
    EXPECT_EQ(result.version, "");
    EXPECT_EQ(result.time, "");
}

TEST(XmlResponseParser, ParseGetStatusResponseWithIncompleteResponse) {
    std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
</Response>)";

    ResponseInfo result = parseResponse(response);

    EXPECT_EQ(result.operation, "");
    EXPECT_EQ(result.status, "");
    EXPECT_EQ(result.message, "");
    EXPECT_EQ(result.version, "");
    EXPECT_EQ(result.time, "");
}
