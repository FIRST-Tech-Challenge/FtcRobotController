#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawEdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * EdgeDetectorConfig message. Carries sobel edge filter config.
 */
class EdgeDetectorConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawEdgeDetectorConfig& cfg;

   public:
    /**
     * Construct EdgeDetectorConfig message.
     */
    EdgeDetectorConfig();
    explicit EdgeDetectorConfig(std::shared_ptr<RawEdgeDetectorConfig> ptr);
    virtual ~EdgeDetectorConfig() = default;

    /**
     * Set sobel filter horizontal and vertical 3x3 kernels
     * @param horizontalKernel Used for horizontal gradient computation in 3x3 Sobel filter
     * @param verticalKernel Used for vertical gradient computation in 3x3 Sobel filter
     */
    void setSobelFilterKernels(const std::vector<std::vector<int>>& horizontalKernel, const std::vector<std::vector<int>>& verticalKernel);

    /**
     * Retrieve configuration data for EdgeDetector
     * @returns EdgeDetectorConfigData: sobel filter horizontal and vertical 3x3 kernels
     */
    EdgeDetectorConfigData getConfigData() const;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    EdgeDetectorConfig& set(dai::RawEdgeDetectorConfig config);

    /**
     * Retrieve configuration data for EdgeDetector.
     * @returns config for EdgeDetector
     */
    dai::RawEdgeDetectorConfig get() const;
};

}  // namespace dai
