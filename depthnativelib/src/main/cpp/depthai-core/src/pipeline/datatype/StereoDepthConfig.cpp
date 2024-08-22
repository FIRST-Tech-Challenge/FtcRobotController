#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> StereoDepthConfig::serialize() const {
    return raw;
}

StereoDepthConfig::StereoDepthConfig() : Buffer(std::make_shared<RawStereoDepthConfig>()), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}
StereoDepthConfig::StereoDepthConfig(std::shared_ptr<RawStereoDepthConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}

StereoDepthConfig& StereoDepthConfig::setDepthAlign(AlgorithmControl::DepthAlign align) {
    cfg.algorithmControl.depthAlign = align;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setConfidenceThreshold(int confThr) {
    cfg.costMatching.confidenceThreshold = confThr;
    return *this;
}

int StereoDepthConfig::getConfidenceThreshold() const {
    return cfg.costMatching.confidenceThreshold;
}

StereoDepthConfig& StereoDepthConfig::setMedianFilter(MedianFilter median) {
    cfg.postProcessing.median = median;
    return *this;
}

MedianFilter StereoDepthConfig::getMedianFilter() const {
    return cfg.postProcessing.median;
}

StereoDepthConfig& StereoDepthConfig::setBilateralFilterSigma(uint16_t sigma) {
    cfg.postProcessing.bilateralSigmaValue = sigma;
    return *this;
}

uint16_t StereoDepthConfig::getBilateralFilterSigma() const {
    return cfg.postProcessing.bilateralSigmaValue;
}

StereoDepthConfig& StereoDepthConfig::setLeftRightCheckThreshold(int threshold) {
    cfg.algorithmControl.leftRightCheckThreshold = threshold;
    return *this;
}

int StereoDepthConfig::getLeftRightCheckThreshold() const {
    return cfg.algorithmControl.leftRightCheckThreshold;
}

StereoDepthConfig& StereoDepthConfig::setLeftRightCheck(bool enable) {
    cfg.algorithmControl.enableLeftRightCheck = enable;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setExtendedDisparity(bool enable) {
    cfg.algorithmControl.enableExtended = enable;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setSubpixel(bool enable) {
    cfg.algorithmControl.enableSubpixel = enable;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setSubpixelFractionalBits(int subpixelFractionalBits) {
    cfg.algorithmControl.subpixelFractionalBits = subpixelFractionalBits;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setDepthUnit(AlgorithmControl::DepthUnit depthUnit) {
    cfg.algorithmControl.depthUnit = depthUnit;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setDisparityShift(int disparityShift) {
    cfg.algorithmControl.disparityShift = disparityShift;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setNumInvalidateEdgePixels(int32_t numInvalidateEdgePixels) {
    cfg.algorithmControl.numInvalidateEdgePixels = numInvalidateEdgePixels;
    return *this;
}

dai::StereoDepthConfig::AlgorithmControl::DepthUnit StereoDepthConfig::getDepthUnit() {
    return cfg.algorithmControl.depthUnit;
}

float StereoDepthConfig::getMaxDisparity() const {
    float maxDisp = 95.0;
    if(cfg.costMatching.disparityWidth == RawStereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64) {
        maxDisp = 63;
    }
    if(cfg.costMatching.enableCompanding) maxDisp = 175;
    maxDisp += cfg.algorithmControl.disparityShift;
    if(cfg.algorithmControl.enableExtended) maxDisp *= 2;
    if(cfg.algorithmControl.enableSubpixel) maxDisp *= (1 << cfg.algorithmControl.subpixelFractionalBits);
    return maxDisp;
}

dai::RawStereoDepthConfig StereoDepthConfig::get() const {
    return cfg;
}

StereoDepthConfig& StereoDepthConfig::set(dai::RawStereoDepthConfig config) {
    cfg = config;
    return *this;
}

}  // namespace dai
