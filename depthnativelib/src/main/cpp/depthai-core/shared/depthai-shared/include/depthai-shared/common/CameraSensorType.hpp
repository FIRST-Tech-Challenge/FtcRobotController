#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/// Camera sensor type
enum class CameraSensorType : int32_t { AUTO = -1, COLOR = 0, MONO = 1, TOF = 2, THERMAL = 3 };

}  // namespace dai
