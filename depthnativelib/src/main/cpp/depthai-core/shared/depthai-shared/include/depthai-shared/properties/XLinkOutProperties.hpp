#pragma once

#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for XLinkOut such as stream name, ...
 */
struct XLinkOutProperties : PropertiesSerializable<Properties, XLinkOutProperties> {
    /**
     * Set a limit to how many packets will be sent further to host
     */
    float maxFpsLimit = -1;

    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * Whether to transfer data or only object attributes
     */
    bool metadataOnly = false;
};

DEPTHAI_SERIALIZE_EXT(XLinkOutProperties, maxFpsLimit, streamName, metadataOnly);

}  // namespace dai
