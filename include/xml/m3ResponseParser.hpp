#pragma once

#include <iostream>
#include <string>

namespace xml {

/**
 * @brief Struct to hold the response from the sonar device
 */
struct ResponseInfo {
	/**
	 * @brief Initialize the struct with empty strings
	 */
	ResponseInfo() : operation(""), status(""), message(""), version(""), time("") {}

	std::string operation;
	std::string status;
	std::string message;
	std::string version;
	std::string time;
};

std::ostream &operator<<(std::ostream &os, ResponseInfo const &m);

/**
 * @brief Parse the response from the sonar device
 *
 * @param response_input xml string response from the M3 sonar
 * @param parsed_response parsed response
 * @return int status of the response, see ResponseStatus enum
 */
ResponseInfo parseResponse(const std::string &response);

} // namespace xml
