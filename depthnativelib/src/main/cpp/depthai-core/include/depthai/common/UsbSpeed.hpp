#pragma once

#include <ostream>

#include "depthai-shared/common/UsbSpeed.hpp"

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::UsbSpeed& speed) {
    switch(speed) {
        case dai::UsbSpeed::UNKNOWN:
            out << "UNKNOWN";
            break;
        case dai::UsbSpeed::LOW:
            out << "LOW";
            break;
        case dai::UsbSpeed::FULL:
            out << "FULL";
            break;
        case dai::UsbSpeed::HIGH:
            out << "HIGH";
            break;
        case dai::UsbSpeed::SUPER:
            out << "SUPER";
            break;
        case dai::UsbSpeed::SUPER_PLUS:
            out << "SUPER_PLUS";
            break;
    }
    return out;
}
