#include <iostream>
#include <api/api.hpp>

int main()
{
	std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2023-09-28 18:03:21</Time>
</Response>)";

	m3::api::ResponseInfo result = m3::api::parseResponse(response);
	std::cout << result << std::endl;

	return 0;
}
