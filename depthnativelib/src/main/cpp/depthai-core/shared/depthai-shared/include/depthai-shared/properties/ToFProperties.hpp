#pragma once

#include "depthai-shared/datatype/RawToFConfig.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for ToF
 */
struct ToFProperties : PropertiesSerializable<Properties, ToFProperties> {
    /**
     * Initial ToF config
     */
    RawToFConfig initialConfig;
};

DEPTHAI_SERIALIZE_EXT(ToFProperties, initialConfig);

}  // namespace dai
