#pragma once

#include "depthai-shared/common/EepromData.hpp"

namespace dai
{
namespace utility
{

/// @brief Splits given string by delimiter
/// @param s string to split
/// @param delimiter character by which to split
/// @return vector of split strings
std::vector<std::string> split(const std::string& s, char delimiter);

/// @brief Parses product name from given EepromData combination
/// @param eeprom EepromData containing fields to parse product name from
/// @param eepromFactory Additional factory eeprom which takes precedence when parsing
/// @return string contaning product name or empty
std::string parseProductName(EepromData eeprom, EepromData eepromFactory = {});

/// @brief Parses device name from given EepromData combination
/// @param eeprom EepromData containing fields to parse device name from
/// @param eepromFactory Additional factory eeprom which takes precedence when parsing
/// @return string contaning device name or empty
std::string parseDeviceName(EepromData eeprom, EepromData eepromFactory = {});

} // namespace utility
} // namespace dai
