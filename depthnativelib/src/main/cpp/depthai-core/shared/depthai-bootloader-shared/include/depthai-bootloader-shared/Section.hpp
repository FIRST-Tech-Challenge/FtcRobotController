#pragma once

// std
#include <cstdint>

namespace dai
{
namespace bootloader
{

enum class Section : std::int32_t {
    AUTO = -1, HEADER = 0, BOOTLOADER = 1, BOOTLOADER_CONFIG = 2, APPLICATION = 3, USER_BOOTLOADER = 4
};

} // namespace bootloader
} // namespace dai

