#pragma once

#include <cstdint>

namespace dai {

/**
 * Interpolation type
 */
enum class Interpolation : std::int32_t {

    AUTO = -1,
    BILINEAR = 0,
    BICUBIC = 1,
    NEAREST_NEIGHBOR = 2,
    BYPASS = NEAREST_NEIGHBOR,
    DEFAULT = BICUBIC,
    DEFAULT_DISPARITY_DEPTH = NEAREST_NEIGHBOR

};

}  // namespace dai
