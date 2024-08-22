#include "depthai/pipeline/datatype/Tracklets.hpp"

namespace dai {

std::shared_ptr<RawBuffer> Tracklets::serialize() const {
    return raw;
}

Tracklets::Tracklets() : Buffer(std::make_shared<RawTracklets>()), rawdata(*dynamic_cast<RawTracklets*>(raw.get())), tracklets(rawdata.tracklets) {}
Tracklets::Tracklets(std::shared_ptr<RawTracklets> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTracklets*>(raw.get())), tracklets(rawdata.tracklets) {}

// setters
Tracklets& Tracklets::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<Tracklets&>(Buffer::setTimestamp(tp));
}
Tracklets& Tracklets::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<Tracklets&>(Buffer::setTimestampDevice(tp));
}
Tracklets& Tracklets::setSequenceNum(int64_t sequenceNum) {
    return static_cast<Tracklets&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai
