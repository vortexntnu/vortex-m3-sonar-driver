#include <iostream>
#include "XmlCommandGenerator.hpp"
#include "XmlResponseParser.hpp"

int main() {
    std::string response = R"(
<Response>
<Operation>Get_Status</Operation>
<Status>OK</Status>
<Message>Pinging</Message>
<Version>2.1.0</Version>
<Time>2016-09-13 15:22:21</Time>
</Response>)";

    ResponseInfo result = parseResponse(response);

    std::cout << "Operation: " << result.operation << std::endl;
    std::cout << "Status: " << result.status << std::endl;
    std::cout << "Message: " << result.message << std::endl;
    std::cout << "Version: " << result.version << std::endl;
    std::cout << "Time: " << result.time << std::endl;

    std::string xmlCommand = XmlCommandGenerator::GenerateXmlCommand("Set_TVG", "", 20, 100, -16, 100);
    std::cout << xmlCommand;

    return 0;
}
