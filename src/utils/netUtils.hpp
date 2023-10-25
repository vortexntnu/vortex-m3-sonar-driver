#pragma once
#include <string>

namespace m3 {
namespace utils {


/**
 * @brief Function to check if the host system is little-endian
 * @return true if the system is little-endian
 */
bool is_little_endian();

/**
 * @brief Function to convert double from network byte order to host byte order
 * 
 * @param byteArray in network byte order (big-endian)
 * @return double 
 */
uint64_t ntohd(uint64_t byteArray);

// This function converts a hex string to a byte array and expects the caller to provide 
// an array that is large enough to hold all the bytes.
void hexStringToByteArray(const std::string& hex_string, unsigned char* byte_array, int size);

} // namespace utils
} // namespace m3