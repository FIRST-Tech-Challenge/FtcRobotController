#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/// Camera sensor type
enum class Colormap : int32_t {
    NONE = 0,
    TURBO,
    JET,
    STEREO_TURBO,
    STEREO_JET
    // AUTUMN,
    // BONE,
    // JET,
    // WINTER,
    // RAINBOW,
    // OCEAN,
    // SUMMER,
    // SPRING,
    // COOL,
    // HSV,
    // PINK,
    // HOT,
    // PARULA,
    // MAGMA,
    // INFERNO,
    // PLASMA,
    // VIRIDIS,
    // CIVIDIS,
    // TWILIGHT,
    // TWILIGHT_SHIFTED,
    // DEEPGREEN
};

}  // namespace dai
