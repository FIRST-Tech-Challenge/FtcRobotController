#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {

std::shared_ptr<RawBuffer> TrackedFeatures::serialize() const {
    return raw;
}

TrackedFeatures::TrackedFeatures()
    : Buffer(std::make_shared<RawTrackedFeatures>()), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}
TrackedFeatures::TrackedFeatures(std::shared_ptr<RawTrackedFeatures> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}

// setters
TrackedFeatures& TrackedFeatures::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<TrackedFeatures&>(Buffer::setTimestamp(tp));
}
TrackedFeatures& TrackedFeatures::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<TrackedFeatures&>(Buffer::setTimestampDevice(tp));
}
TrackedFeatures& TrackedFeatures::setSequenceNum(int64_t sequenceNum) {
    return static_cast<TrackedFeatures&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai
