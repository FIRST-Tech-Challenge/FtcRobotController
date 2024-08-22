#pragma once

#include <vector>

#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for SpatialLocationCalculator
 */
struct SpatialLocationCalculatorProperties : PropertiesSerializable<Properties, SpatialLocationCalculatorProperties> {
    RawSpatialLocationCalculatorConfig roiConfig;
};

DEPTHAI_SERIALIZE_EXT(SpatialLocationCalculatorProperties, roiConfig);

}  // namespace dai
