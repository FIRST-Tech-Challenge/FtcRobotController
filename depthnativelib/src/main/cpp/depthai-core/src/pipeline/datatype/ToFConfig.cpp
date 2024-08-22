#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> ToFConfig::serialize() const {
    return raw;
}

ToFConfig::ToFConfig() : Buffer(std::make_shared<RawToFConfig>()), cfg(*dynamic_cast<RawToFConfig*>(raw.get())) {}
ToFConfig::ToFConfig(std::shared_ptr<RawToFConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawToFConfig*>(raw.get())) {}

dai::RawToFConfig ToFConfig::get() const {
    return cfg;
}

ToFConfig& ToFConfig::setDepthParams(dai::ToFConfig::DepthParams config) {
    cfg.depthParams = config;
    return *this;
}

ToFConfig& ToFConfig::setFreqModUsed(dai::ToFConfig::DepthParams::TypeFMod fmod) {
    cfg.depthParams.freqModUsed = fmod;
    return *this;
}

ToFConfig& ToFConfig::setAvgPhaseShuffle(bool enable) {
    cfg.depthParams.avgPhaseShuffle = enable;
    return *this;
}

ToFConfig& ToFConfig::set(dai::RawToFConfig config) {
    cfg = config;
    return *this;
}

ToFConfig& ToFConfig::setMinAmplitude(float minamp) {
    cfg.depthParams.minimumAmplitude = minamp;
    return *this;
}

ToFConfig& ToFConfig::setMedianFilter(MedianFilter median) {
    cfg.depthParams.median = median;
    return *this;
}

}  // namespace dai
