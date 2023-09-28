#include <iostream>
#include <XmlCommandGenerator.hpp>
#include <XmlResponseParser.hpp>

int main() {
    std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2023-09-28 18:03:21</Time>
</Response>)";

    ResponseInfo result = parseResponse(response);
    std::cout << result << std::endl;

    return 0;
}
