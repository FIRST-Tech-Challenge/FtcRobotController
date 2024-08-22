#pragma once
#include <cstdint>
#include <vector>

#include "RawImgFrame.hpp"
#include "depthai-shared/common/Rect.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// EdgeDetectorConfigData configuration data structure
struct EdgeDetectorConfigData {
    /**
     * Used for horizontal gradient computation in 3x3 Sobel filter
     * Format - 3x3 matrix, 2nd column must be 0
     * Default - +1 0 -1; +2 0 -2; +1 0 -1
     */
    std::vector<std::vector<int>> sobelFilterHorizontalKernel;
    /**
     * Used for vertical gradient computation in 3x3 Sobel filter
     * Format - 3x3 matrix, 2nd row must be 0
     * Default - +1 +2 +1; 0 0 0; -1 -2 -1
     */
    std::vector<std::vector<int>> sobelFilterVerticalKernel;
};
DEPTHAI_SERIALIZE_EXT(EdgeDetectorConfigData, sobelFilterHorizontalKernel, sobelFilterVerticalKernel);

/// RawEdgeDetectorConfig configuration structure
struct RawEdgeDetectorConfig : public RawBuffer {
    EdgeDetectorConfigData config;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::EdgeDetectorConfig;
    };

    DEPTHAI_SERIALIZE(RawEdgeDetectorConfig, config);
};

}  // namespace dai
