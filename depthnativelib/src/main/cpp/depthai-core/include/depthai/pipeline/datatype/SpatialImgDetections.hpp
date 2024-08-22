#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * SpatialImgDetections message. Carries detection results together with spatial location data
 */
class SpatialImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialImgDetections& dets;

   public:
    /**
     * Construct SpatialImgDetections message.
     */
    SpatialImgDetections();
    explicit SpatialImgDetections(std::shared_ptr<RawSpatialImgDetections> ptr);
    virtual ~SpatialImgDetections() = default;

    /**
     * Detection results.
     */
    std::vector<SpatialImgDetection>& detections;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    SpatialImgDetections& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    SpatialImgDetections& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    SpatialImgDetections& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
