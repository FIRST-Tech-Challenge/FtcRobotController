#pragma once

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * DetectionParserOptions
 *
 * Specifies how to parse output of detection networks
 */
struct DetectionParserOptions {
    /// Generic Neural Network properties
    DetectionNetworkType nnFamily;
    float confidenceThreshold;

    /// YOLO specific network properties
    int classes;
    int coordinates;
    std::vector<float> anchors;
    std::map<std::string, std::vector<int>> anchorMasks;
    float iouThreshold;
};

DEPTHAI_SERIALIZE_EXT(DetectionParserOptions, nnFamily, confidenceThreshold, classes, coordinates, anchors, anchorMasks, iouThreshold);

}  // namespace dai