#pragma once

// std
#include <cstdint>

// project
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Point2f structure
 *
 * x and y coordinates that define a 2D point.
 */
struct Point2f {
    Point2f() = default;
    Point2f(float x, float y) : x(x), y(y) {}
    float x = 0, y = 0;
};

DEPTHAI_SERIALIZE_EXT(Point2f, x, y);

}  // namespace dai
