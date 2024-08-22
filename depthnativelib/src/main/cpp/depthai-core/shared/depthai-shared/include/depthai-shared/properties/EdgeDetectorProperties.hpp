#pragma once

#include <vector>

#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/datatype/RawEdgeDetectorConfig.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for EdgeDetector
 */
struct EdgeDetectorProperties : PropertiesSerializable<Properties, EdgeDetectorProperties> {
    /// Initial edge detector config
    RawEdgeDetectorConfig initialConfig;

    /**
     * Maximum output frame size in bytes (eg: 300x300 BGR image -> 300*300*3 bytes)
     */
    int outputFrameSize = 1 * 1024 * 1024;

    /// Num frames in output pool
    int numFramesPool = 4;
};

DEPTHAI_SERIALIZE_EXT(EdgeDetectorProperties, initialConfig, outputFrameSize, numFramesPool);

}  // namespace dai
