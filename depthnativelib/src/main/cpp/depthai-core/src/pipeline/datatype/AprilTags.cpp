#include "depthai/pipeline/datatype/AprilTags.hpp"

namespace dai {

std::shared_ptr<RawBuffer> AprilTags::serialize() const {
    return raw;
}

AprilTags::AprilTags() : Buffer(std::make_shared<RawAprilTags>()), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}
AprilTags::AprilTags(std::shared_ptr<RawAprilTags> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}

// setters
AprilTags& AprilTags::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<AprilTags&>(Buffer::setTimestamp(tp));
}
AprilTags& AprilTags::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<AprilTags&>(Buffer::setTimestampDevice(tp));
}
AprilTags& AprilTags::setSequenceNum(int64_t sequenceNum) {
    return static_cast<AprilTags&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai