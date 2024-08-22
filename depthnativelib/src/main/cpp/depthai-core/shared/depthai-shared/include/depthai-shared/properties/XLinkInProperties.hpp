#pragma once

#include "depthai-shared/properties/Properties.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

namespace dai {

/**
 * Specify properties for XLinkIn such as stream name, ...
 */
struct XLinkInProperties : PropertiesSerializable<Properties, XLinkInProperties> {
    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * Maximum input data size
     */
    std::uint32_t maxDataSize = dai::device::XLINK_USB_BUFFER_MAX_SIZE;

    /**
     * Number of frames in pool
     */
    std::uint32_t numFrames = 8;
};

DEPTHAI_SERIALIZE_EXT(XLinkInProperties, streamName, maxDataSize, numFrames);

}  // namespace dai
