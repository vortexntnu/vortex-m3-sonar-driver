#include <gtest/gtest.h>
#include <rapidxml/rapidxml.hpp>
#include <api/m3ResponseParser.hpp>

TEST(XmlResponseParser, ParseGetStatusResponseWithValidXml)
{
	std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2016-09-13 15:22:21</Time>
</Response>)";

	m3::api::ResponseInfo result = m3::api::parseResponse(response);

	EXPECT_EQ(result.operation, "Get_Status");
	EXPECT_EQ(result.status, "OK");
	EXPECT_EQ(result.message, "Pinging");
	EXPECT_EQ(result.version, "2.1.0");
	EXPECT_EQ(result.time, "2016-09-13 15:22:21");
}

TEST(XmlResponseParser, ParseGetStatusResponseWithInvalidXml)
{
	std::string response = R"(Invalid XML)";

	try {
		m3::api::ResponseInfo result = m3::api::parseResponse(response);
		FAIL() << "Expected rapidxml::parse_error";
	}
	catch (const rapidxml::parse_error &e) {
		EXPECT_EQ(e.what(), std::string("expected <"));
	}
}

TEST(XmlResponseParser, ParseGetStatusResponseWithInvalidResponse)
{
	std::string response = R"(
<NoResponse>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2016-09-13 15:22:21</Time>
</NoResponse>)";

	try {
		m3::api::ResponseInfo result = m3::api::parseResponse(response);
		FAIL() << "Expected std::runtime_error";
	}
	catch (const std::runtime_error &e) {
		EXPECT_EQ(e.what(), std::string("Invalid response"));
	}
}

TEST(XmlResponseParser, ParseGetStatusResponseWithIncompleteResponse)
{
	std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
</Response>)";

	try {
		m3::api::ResponseInfo result = m3::api::parseResponse(response);
		FAIL() << "Expected std::runtime_error";
	}
	catch (const std::runtime_error &e) {
		EXPECT_EQ(e.what(), std::string("Incomplete response"));
	}
	catch (const std::exception &e) {
		std::cerr << e.what() << '\n';
	}
}