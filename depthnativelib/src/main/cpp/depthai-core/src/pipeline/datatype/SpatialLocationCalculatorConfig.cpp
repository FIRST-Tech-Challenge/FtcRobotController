#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialLocationCalculatorConfig::serialize() const {
    return raw;
}

SpatialLocationCalculatorConfig::SpatialLocationCalculatorConfig()
    : Buffer(std::make_shared<RawSpatialLocationCalculatorConfig>()), cfg(*dynamic_cast<RawSpatialLocationCalculatorConfig*>(raw.get())) {}
SpatialLocationCalculatorConfig::SpatialLocationCalculatorConfig(std::shared_ptr<RawSpatialLocationCalculatorConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawSpatialLocationCalculatorConfig*>(raw.get())) {}

void SpatialLocationCalculatorConfig::setROIs(std::vector<SpatialLocationCalculatorConfigData> ROIs) {
    cfg.config = ROIs;
}

void SpatialLocationCalculatorConfig::addROI(SpatialLocationCalculatorConfigData& ROI) {
    cfg.config.push_back(ROI);
}

std::vector<SpatialLocationCalculatorConfigData> SpatialLocationCalculatorConfig::getConfigData() const {
    return cfg.config;
}

dai::RawSpatialLocationCalculatorConfig SpatialLocationCalculatorConfig::get() const {
    return cfg;
}

SpatialLocationCalculatorConfig& SpatialLocationCalculatorConfig::set(dai::RawSpatialLocationCalculatorConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai
