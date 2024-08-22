#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> FeatureTrackerConfig::serialize() const {
    return raw;
}

FeatureTrackerConfig::FeatureTrackerConfig() : Buffer(std::make_shared<RawFeatureTrackerConfig>()), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}
FeatureTrackerConfig::FeatureTrackerConfig(std::shared_ptr<RawFeatureTrackerConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawFeatureTrackerConfig*>(raw.get())) {}

dai::RawFeatureTrackerConfig FeatureTrackerConfig::get() const {
    return cfg;
}

FeatureTrackerConfig& FeatureTrackerConfig::setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type cornerDetector) {
    cfg.cornerDetector.type = cornerDetector;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setCornerDetector(dai::FeatureTrackerConfig::CornerDetector config) {
    cfg.cornerDetector = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setMotionEstimator(bool enable) {
    cfg.motionEstimator.enable = enable;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setMotionEstimator(dai::FeatureTrackerConfig::MotionEstimator config) {
    cfg.motionEstimator = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setOpticalFlow() {
    cfg.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    setMotionEstimator(true);
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setOpticalFlow(dai::FeatureTrackerConfig::MotionEstimator::OpticalFlow config) {
    cfg.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
    cfg.motionEstimator.opticalFlow = config;
    setMotionEstimator(true);
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setHwMotionEstimation() {
    cfg.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION;
    setMotionEstimator(true);
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setFeatureMaintainer(bool enable) {
    cfg.featureMaintainer.enable = enable;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setFeatureMaintainer(dai::FeatureTrackerConfig::FeatureMaintainer config) {
    cfg.featureMaintainer = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::set(dai::RawFeatureTrackerConfig config) {
    cfg = config;
    return *this;
}

FeatureTrackerConfig& FeatureTrackerConfig::setNumTargetFeatures(std::int32_t numTargetFeatures) {
    cfg.cornerDetector.numTargetFeatures = numTargetFeatures;
    return *this;
}

}  // namespace dai
