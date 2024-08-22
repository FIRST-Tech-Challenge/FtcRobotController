#pragma once

// std
#include <cstdint>

// shared
#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/Size2f.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RotatedRect structure
struct RotatedRect {
    Point2f center;
    Size2f size;
    /// degrees, increasing clockwise
    float angle = 0.f;
};

DEPTHAI_SERIALIZE_EXT(RotatedRect, center, size, angle);

}  // namespace dai
