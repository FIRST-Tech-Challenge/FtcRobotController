#pragma once

// std
#include <cstdint>

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Size2f structure
 *
 * width, height values define the size of the shape/frame
 */
struct Size2f {
    Size2f() = default;
    Size2f(float width, float height) : width(width), height(height) {}
    float width = 0, height = 0;
};

DEPTHAI_SERIALIZE_EXT(Size2f, width, height);

}  // namespace dai
