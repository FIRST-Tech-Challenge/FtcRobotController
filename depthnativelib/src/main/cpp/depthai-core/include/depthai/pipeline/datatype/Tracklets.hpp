#pragma once

#include <ostream>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTracklets.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Tracklets message. Carries object tracking information.
 */
class Tracklets : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawTracklets& rawdata;

   public:
    /**
     * Construct Tracklets message.
     */
    Tracklets();
    explicit Tracklets(std::shared_ptr<RawTracklets> ptr);
    virtual ~Tracklets() = default;

    /**
     * Retrieve data for Tracklets.
     * @returns Vector of object tracker data, carrying tracking information.
     */
    std::vector<Tracklet>& tracklets;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    Tracklets& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    Tracklets& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    Tracklets& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::Tracklet::TrackingStatus& status) {
    switch(status) {
        case dai::Tracklet::TrackingStatus::NEW:
            out << "NEW";
            break;
        case dai::Tracklet::TrackingStatus::TRACKED:
            out << "TRACKED";
            break;
        case dai::Tracklet::TrackingStatus::LOST:
            out << "LOST";
            break;
        case dai::Tracklet::TrackingStatus::REMOVED:
            out << "REMOVED";
            break;
    }
    return out;
}