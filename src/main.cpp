#include <iostream>
#include <xml/xml.hpp>

int main() {
    std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2023-09-28 18:03:21</Time>
</Response>)";

    xml::ResponseInfo result = xml::parseResponse(response);
    std::cout << result << std::endl;

    return 0;
}
