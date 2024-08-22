#pragma once

// std
#include <cstdint>

// project
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Point3f structure
 *
 * x,y,z coordinates that define a 3D point.
 */
struct Point3f {
    Point3f() = default;
    Point3f(float x, float y, float z) : x(x), y(y), z(z) {}
    float x = 0, y = 0, z = 0;
};

DEPTHAI_SERIALIZE_EXT(Point3f, x, y, z);

}  // namespace dai
