#ifndef XMLRESPONSEPARSER_HPP
#define XMLRESPONSEPARSER_HPP

#include <string>

struct ResponseInfo {
    std::string operation;
    std::string status;
    std::string message;
    std::string version;
    std::string time;

    // Initialize all members to empty string
    ResponseInfo() : operation(""), status(""), message(""), version(""), time("") {}
};

enum class ResponseStatus {
    OK = 0,
    INCOMPLETE_RESPONSE,
    INVALID_RESPONSE
};

/**
 * @brief Parse the response from the sonar device
 * 
 * @param response_input xml string response from the sonar device
 * @param parsed_response parsed response
 * @return int status of the response, see ResponseStatus enum
 */
int parseResponse(const std::string& response_input, ResponseInfo& parsed_response);

#endif // XMLRESPONSEPARSER_HPP
