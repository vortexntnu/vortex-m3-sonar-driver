#pragma once

#include <iostream>
#include <string>

namespace m3 {
namespace api {

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
 * @param response xml string response from the M3 sonar
 * @return ResponseInfo parsed response
 * @throw std::runtime_error if the response is invalid
 * @throw std::runtime_error if the response is incomplete
 * @throw rapidxml::parse_error if the response is not a valid XML
 */
ResponseInfo parseResponse(const std::string &response);

} // namespace api
} // namespace m3