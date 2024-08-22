#pragma once

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraImageOrientation.hpp"
#include "depthai-shared/common/CameraSensorType.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Sensor config
 */
struct CameraSensorConfig {
    std::int32_t width = -1, height = -1;
    std::int32_t minFps = -1, maxFps = -1;
    CameraSensorType type;
};
DEPTHAI_SERIALIZE_EXT(CameraSensorConfig, width, height, minFps, maxFps, type);

/**
 * CameraFeatures structure
 *
 * Characterizes detected cameras on board
 */
struct CameraFeatures {
    /**
     * Board socket where the camera was detected
     */
    CameraBoardSocket socket = CameraBoardSocket::AUTO;
    /**
     * Camera sensor name, e.g: "IMX378", "OV9282"
     */
    std::string sensorName;
    /**
     * Maximum sensor resolution
     */
    std::int32_t width = -1, height = -1;
    /**
     * Default camera orientation, board dependent
     */
    CameraImageOrientation orientation = CameraImageOrientation::AUTO;
    /**
     * List of supported types of processing for the given camera.
     *
     * For some sensors it's not possible to determine if they are color or mono
     * (e.g. OV9782 and OV9282), so this could return more than one entry
     */
    std::vector<CameraSensorType> supportedTypes;
    /**
     *  Whether an autofocus VCM IC was detected
     */
    bool hasAutofocusIC = false;
    /**
     *  Whether camera has auto focus capabilities, or is a fixed focus lens
     */
    bool hasAutofocus = false;
    /**
     * Camera name or alias
     */
    std::string name;
    /**
     * Additional camera names or aliases
     */
    std::vector<std::string> additionalNames;
    /**
     * Available sensor configs
     */
    std::vector<CameraSensorConfig> configs;

    DEPTHAI_SERIALIZE(
        CameraFeatures, socket, sensorName, width, height, orientation, supportedTypes, hasAutofocusIC, hasAutofocus, name, additionalNames, configs);
};

}  // namespace dai
