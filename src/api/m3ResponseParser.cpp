#include <iostream>
#include <rapidxml/rapidxml.hpp>
#include <api/m3ResponseParser.hpp>

namespace m3 {
namespace api{

ResponseInfo parseResponse(const std::string &response)
{
	rapidxml::xml_document<> doc;
	doc.parse<0>(const_cast<char *>(response.c_str()));

	rapidxml::xml_node<> *root = doc.first_node("Response");

	if (!root) {
		throw std::runtime_error("Invalid response");
	}

	rapidxml::xml_node<> *operationNode = root->first_node("Operation");
	rapidxml::xml_node<> *statusNode    = root->first_node("Status");
	rapidxml::xml_node<> *messageNode   = root->first_node("Message");
	rapidxml::xml_node<> *versionNode   = root->first_node("Version");
	rapidxml::xml_node<> *timeNode      = root->first_node("Time");

	if (!operationNode || !statusNode || !messageNode || !timeNode) {
		throw std::runtime_error("Incomplete response");
	}

	ResponseInfo info;
	info.operation = operationNode->value();
	info.status    = statusNode->value();
	info.message   = messageNode->value();
	info.version   = (versionNode) ? versionNode->value() : "";
	info.time      = timeNode->value();

	return info;
}

std::ostream &operator<<(std::ostream &os, ResponseInfo const &m)
{
	return os << "operation: " << m.operation << '\n'
	          << "status: " << m.status << '\n'
	          << "message: " << m.message << '\n'
	          << "version: " << m.version << '\n'
	          << "time: " << m.time << '\n';
}

} // namespace api
} // namespace m3
