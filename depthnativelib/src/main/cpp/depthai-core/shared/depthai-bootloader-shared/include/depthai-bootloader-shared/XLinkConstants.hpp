#pragma once

#include <chrono>
#include <cstdint>

namespace dai {
namespace bootloader
{
    
// channel names
constexpr static const char* XLINK_CHANNEL_BOOTLOADER = "__bootloader";
constexpr static const char* XLINK_CHANNEL_WATCHDOG = "__watchdog";

// Stream maximum size
constexpr static std::uint32_t XLINK_STREAM_MAX_SIZE = 5 * 1024 * 1024;

// Watchdog timeout
constexpr static const std::chrono::milliseconds XLINK_WATCHDOG_TIMEOUT{1500};

} // namespace bootloader
}  // namespace dai
