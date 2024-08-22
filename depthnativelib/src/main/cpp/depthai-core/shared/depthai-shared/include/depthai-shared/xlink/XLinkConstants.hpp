#pragma once

#include <chrono>
#include <cstdint>

namespace dai {
namespace device {

// channel names
constexpr static const char* XLINK_CHANNEL_PIPELINE_CONFIG = "__pipeline_config";
constexpr static const char* XLINK_CHANNEL_MAIN_RPC = "__rpc_main";
constexpr static const char* XLINK_CHANNEL_TIMESYNC = "__timesync";
constexpr static const char* XLINK_CHANNEL_LOG = "__log";
constexpr static const char* XLINK_CHANNEL_WATCHDOG = "__watchdog";

// usb buffer maximum size
constexpr static std::uint32_t XLINK_USB_BUFFER_MAX_SIZE = 5 * 1024 * 1024;

// Watchdog timeout
constexpr static const std::chrono::milliseconds XLINK_USB_WATCHDOG_TIMEOUT{1500};
constexpr static const std::chrono::milliseconds XLINK_TCP_WATCHDOG_TIMEOUT{4000};

// Maximum metadata size
constexpr static const int XLINK_MESSAGE_METADATA_MAX_SIZE = 50 * 1024;

}  // namespace device
}  // namespace dai
