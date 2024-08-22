#pragma once

// std
#include <cstdint>

namespace dai {

/**
 * Get USB Speed
 */

enum class UsbSpeed : int32_t { UNKNOWN, LOW, FULL, HIGH, SUPER, SUPER_PLUS };

}  // namespace dai
