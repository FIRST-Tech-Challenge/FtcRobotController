#pragma once

// std
#include <cstdint>

// project
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

// Follows spdlog levels
enum class LogLevel : std::int32_t { TRACE = 0, DEBUG, INFO, WARN, ERR, CRITICAL, OFF };

}  // namespace dai
