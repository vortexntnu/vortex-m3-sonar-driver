#include "netUtils.hpp"
#include <cctype> // for std::isspace
#include <cstdlib> // for std::strtol
#include <algorithm> // for std::remove_if
#include <stdexcept> // for std::invalid_argument

bool is_little_endian() 
{
    uint16_t number = 0x1;
    char *numPtr = (char*)&number;
    return (numPtr[0] == 1);
}

uint64_t ntohd(uint64_t byteArray) 
{
    if (is_little_endian()) {
        return __builtin_bswap64(byteArray);
    } 
    return byteArray;
}



void hexStringToByteArray(const std::string& hex_string, unsigned char* byte_array, int size) {
    // Create a copy of the input string and remove whitespace and line breaks
    std::string processed_hex_string = hex_string;
    processed_hex_string.erase(std::remove_if(processed_hex_string.begin(), processed_hex_string.end(), [](unsigned char c){
        return std::isspace(c);
    }), processed_hex_string.end());

    // Ensure the processed string has an even length
    if (processed_hex_string.length() % 2 != 0) {
        // handle error, e.g., throw exception, return, assert, etc.
        throw std::invalid_argument("Invalid hex string length after removing whitespace. Must be even.");
        return;
    }

    // Adjust the size if the processed string length is shorter than expected
    int processed_size = std::min(size, (int)processed_hex_string.length() / 2);

    for (int i = 0; i < processed_size; ++i) {
        std::string byteString = processed_hex_string.substr(i * 2, 2);
        uint8_t byte = (uint8_t)std::strtol(byteString.c_str(), nullptr, 16);
        byte_array[i] = byte;
    }

    // If the original size is larger, fill in the rest with zeros or handle accordingly
    for (int i = processed_size; i < size; ++i) {
        byte_array[i] = 0;
    }
}