#include "XmlResponseParser.hpp"
#include "rapidxml.hpp"

int parseResponse(const std::string& response_input, ResponseInfo& parsed_response) {
    rapidxml::xml_document<> doc;
    doc.parse<0>(const_cast<char*>(response_input.c_str()));

    rapidxml::xml_node<>* root = doc.first_node("Response");


    if (!root) {
        std::cerr << "Invalid response format" << std::endl;
        return ResponseStatus::INVALID_RESPONSE;
    }

    rapidxml::xml_node<>* operationNode = root->first_node("Operation");
    rapidxml::xml_node<>* statusNode = root->first_node("Status");
    rapidxml::xml_node<>* messageNode = root->first_node("Message");
    rapidxml::xml_node<>* versionNode = root->first_node("Version");
    rapidxml::xml_node<>* timeNode = root->first_node("Time");

    if (!operationNode || !statusNode || !messageNode || !timeNode) {
        std::cerr << "Incomplete response" << std::endl;
        return ResponseStatus::INCOMPLETE_RESPONSE;
    }

    info.operation = operationNode->value();
    info.status = statusNode->value();
    info.message = messageNode->value();
    info.version = (versionNode) ? versionNode->value() : "";
    info.time = timeNode->value();

    return info;
}
