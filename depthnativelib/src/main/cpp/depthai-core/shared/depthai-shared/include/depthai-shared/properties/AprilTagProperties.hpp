#pragma once

#include <depthai-shared/datatype/RawAprilTagConfig.hpp>
#include <vector>

#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for AprilTag
 */
struct AprilTagProperties : PropertiesSerializable<Properties, AprilTagProperties> {
    RawAprilTagConfig initialConfig;

    /// Whether to wait for config at 'inputConfig' IO
    bool inputConfigSync = false;
};

DEPTHAI_SERIALIZE_EXT(AprilTagProperties, initialConfig, inputConfigSync);

}  // namespace dai
