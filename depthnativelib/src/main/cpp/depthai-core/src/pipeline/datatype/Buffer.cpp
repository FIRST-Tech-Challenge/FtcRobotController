#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

std::shared_ptr<dai::RawBuffer> Buffer::serialize() const {
    return raw;
}

Buffer::Buffer() : ADatatype(std::make_shared<dai::RawBuffer>()) {}
Buffer::Buffer(std::shared_ptr<dai::RawBuffer> ptr) : ADatatype(std::move(ptr)) {}

// helpers
std::vector<std::uint8_t>& Buffer::getData() const {
    return raw->data;
}

void Buffer::setData(const std::vector<std::uint8_t>& data) {
    raw->data = data;
}

void Buffer::setData(std::vector<std::uint8_t>&& data) {
    raw->data = std::move(data);
}

// getters
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestamp() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(raw->ts.sec) + nanoseconds(raw->ts.nsec)};
}
std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> Buffer::getTimestampDevice() const {
    using namespace std::chrono;
    return time_point<steady_clock, steady_clock::duration>{seconds(raw->tsDevice.sec) + nanoseconds(raw->tsDevice.nsec)};
}
int64_t Buffer::getSequenceNum() const {
    return raw->sequenceNum;
}

// setters
Buffer& Buffer::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    raw->ts.sec = duration_cast<seconds>(ts).count();
    raw->ts.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
Buffer& Buffer::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    using namespace std::chrono;
    auto ts = tp.time_since_epoch();
    raw->tsDevice.sec = duration_cast<seconds>(ts).count();
    raw->tsDevice.nsec = duration_cast<nanoseconds>(ts).count() % 1000000000;
    return *this;
}
Buffer& Buffer::setSequenceNum(int64_t sequenceNum) {
    raw->sequenceNum = sequenceNum;
    return *this;
}

}  // namespace dai
