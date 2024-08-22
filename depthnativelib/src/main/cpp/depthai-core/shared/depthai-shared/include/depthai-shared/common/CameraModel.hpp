#pragma once

#include <cstdint>

namespace dai {
/**
 * Which CameraModel to initialize the calibration with.
 */
enum class CameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 };

}  // namespace dai