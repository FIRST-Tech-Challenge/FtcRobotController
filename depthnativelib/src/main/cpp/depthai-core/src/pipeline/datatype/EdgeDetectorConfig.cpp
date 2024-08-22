#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> EdgeDetectorConfig::serialize() const {
    return raw;
}

EdgeDetectorConfig::EdgeDetectorConfig() : Buffer(std::make_shared<RawEdgeDetectorConfig>()), cfg(*dynamic_cast<RawEdgeDetectorConfig*>(raw.get())) {}
EdgeDetectorConfig::EdgeDetectorConfig(std::shared_ptr<RawEdgeDetectorConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawEdgeDetectorConfig*>(raw.get())) {}

void EdgeDetectorConfig::setSobelFilterKernels(const std::vector<std::vector<int>>& horizontalKernel, const std::vector<std::vector<int>>& verticalKernel) {
    cfg.config.sobelFilterHorizontalKernel = horizontalKernel;
    cfg.config.sobelFilterVerticalKernel = verticalKernel;
}

EdgeDetectorConfigData EdgeDetectorConfig::getConfigData() const {
    return cfg.config;
}

dai::RawEdgeDetectorConfig EdgeDetectorConfig::get() const {
    return cfg;
}

EdgeDetectorConfig& EdgeDetectorConfig::set(dai::RawEdgeDetectorConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai
