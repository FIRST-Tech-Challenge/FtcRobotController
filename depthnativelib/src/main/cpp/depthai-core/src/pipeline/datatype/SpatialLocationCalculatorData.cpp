#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialLocationCalculatorData::serialize() const {
    return raw;
}

SpatialLocationCalculatorData::SpatialLocationCalculatorData()
    : Buffer(std::make_shared<RawSpatialLocations>()), rawdata(*dynamic_cast<RawSpatialLocations*>(raw.get())), spatialLocations(rawdata.spatialLocations) {}
SpatialLocationCalculatorData::SpatialLocationCalculatorData(std::shared_ptr<RawSpatialLocations> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawSpatialLocations*>(raw.get())), spatialLocations(rawdata.spatialLocations) {}

std::vector<SpatialLocations>& SpatialLocationCalculatorData::getSpatialLocations() const {
    return rawdata.spatialLocations;
}

// setters
SpatialLocationCalculatorData& SpatialLocationCalculatorData::setTimestamp(
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<SpatialLocationCalculatorData&>(Buffer::setTimestamp(tp));
}
SpatialLocationCalculatorData& SpatialLocationCalculatorData::setTimestampDevice(
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<SpatialLocationCalculatorData&>(Buffer::setTimestampDevice(tp));
}
SpatialLocationCalculatorData& SpatialLocationCalculatorData::setSequenceNum(int64_t sequenceNum) {
    return static_cast<SpatialLocationCalculatorData&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai
