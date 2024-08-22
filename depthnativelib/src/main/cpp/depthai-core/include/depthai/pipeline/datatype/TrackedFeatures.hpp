#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * TrackedFeatures message. Carries position (X, Y) of tracked features and their ID.
 */
class TrackedFeatures : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawTrackedFeatures& rawdata;

   public:
    /**
     * Construct TrackedFeatures message.
     */
    TrackedFeatures();
    explicit TrackedFeatures(std::shared_ptr<RawTrackedFeatures> ptr);
    virtual ~TrackedFeatures() = default;

    std::vector<TrackedFeature>& trackedFeatures;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    TrackedFeatures& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    TrackedFeatures& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    TrackedFeatures& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
