#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTags message.
 */
class AprilTags : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAprilTags& rawdata;

   public:
    /**
     * Construct AprilTags message.
     */
    AprilTags();
    explicit AprilTags(std::shared_ptr<RawAprilTags> ptr);
    virtual ~AprilTags() = default;

    std::vector<AprilTag>& aprilTags;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    AprilTags& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    AprilTags& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    AprilTags& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
