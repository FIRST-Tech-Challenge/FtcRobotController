#pragma once

// std
#include <cstdint>

namespace dai
{
namespace bootloader
{

enum class Memory : std::int32_t {
    AUTO = -1, FLASH = 0, EMMC = 1,
};

} // namespace bootloader
} // namespace dai
