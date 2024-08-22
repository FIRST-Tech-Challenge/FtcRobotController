#pragma once

#include "depthai-shared/properties/Properties.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

namespace dai {

/**
 * Properties for SPIIn node
 */
struct SPIInProperties : PropertiesSerializable<Properties, SPIInProperties> {
    /**
     * Name of stream
     */
    std::string streamName;

    /**
     * SPI bus to use
     */
    int busId = 0;

    /**
     * Maximum input data size
     */
    std::uint32_t maxDataSize = dai::device::XLINK_USB_BUFFER_MAX_SIZE;

    /**
     * Number of frames in pool
     */
    std::uint32_t numFrames = 4;
};

DEPTHAI_SERIALIZE_EXT(SPIInProperties, streamName, busId, maxDataSize, numFrames);

}  // namespace dai
