#pragma once

// std
#include <cstdint>

namespace dai
{
namespace bootloader
{

enum class Type : std::int32_t {
    AUTO = -1, USB = 0, NETWORK = 1
};

} // namespace bootloader
} // namespace dai
