#pragma once

#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Properties for UVC node
 */
struct UVCProperties : PropertiesSerializable<Properties, UVCProperties> {
    /// <gpio_number, value> list for GPIOs to set at init
    std::unordered_map<int, int> gpioInit;

    /// <gpio_number, value> list for GPIOs to set when streaming is enabled
    std::unordered_map<int, int> gpioStreamOn;

    /// <gpio_number, value> list for GPIOs to set when streaming is disabled
    std::unordered_map<int, int> gpioStreamOff;
};

DEPTHAI_SERIALIZE_EXT(UVCProperties, gpioInit, gpioStreamOn, gpioStreamOff);

}  // namespace dai
