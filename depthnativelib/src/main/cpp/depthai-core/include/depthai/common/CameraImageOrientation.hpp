#pragma once

#include <ostream>

#include "depthai-shared/common/CameraImageOrientation.hpp"

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraImageOrientation& orientation) {
    switch(orientation) {
        case dai::CameraImageOrientation::AUTO:
            out << "AUTO";
            break;
        case dai::CameraImageOrientation::NORMAL:
            out << "NORMAL";
            break;
        case dai::CameraImageOrientation::HORIZONTAL_MIRROR:
            out << "HORIZONTAL_MIRROR";
            break;
        case dai::CameraImageOrientation::VERTICAL_FLIP:
            out << "VERTICAL_FLIP";
            break;
        case dai::CameraImageOrientation::ROTATE_180_DEG:
            out << "ROTATE_180_DEG";
            break;
    }
    return out;
}
