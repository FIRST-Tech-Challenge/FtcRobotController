#pragma once

#include <ostream>

#include "depthai-shared/common/CameraSensorType.hpp"

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraSensorType& type) {
    switch(type) {
        case dai::CameraSensorType::AUTO:
            out << "AUTO";
            break;
        case dai::CameraSensorType::COLOR:
            out << "COLOR";
            break;
        case dai::CameraSensorType::MONO:
            out << "MONO";
            break;
        case dai::CameraSensorType::TOF:
            out << "TOF";
            break;
        case dai::CameraSensorType::THERMAL:
            out << "THERMAL";
            break;
    }
    return out;
}
