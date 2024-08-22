#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SpatialImgDetections::serialize() const {
    return raw;
}

SpatialImgDetections::SpatialImgDetections()
    : Buffer(std::make_shared<RawSpatialImgDetections>()), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}
SpatialImgDetections::SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr)
    : Buffer(std::move(ptr)), dets(*dynamic_cast<RawSpatialImgDetections*>(raw.get())), detections(dets.detections) {}

// setters
SpatialImgDetections& SpatialImgDetections::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<SpatialImgDetections&>(Buffer::setTimestamp(tp));
}
SpatialImgDetections& SpatialImgDetections::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<SpatialImgDetections&>(Buffer::setTimestampDevice(tp));
}
SpatialImgDetections& SpatialImgDetections::setSequenceNum(int64_t sequenceNum) {
    return static_cast<SpatialImgDetections&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai
