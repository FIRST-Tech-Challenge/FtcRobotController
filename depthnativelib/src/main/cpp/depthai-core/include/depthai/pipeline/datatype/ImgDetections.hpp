#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * ImgDetections message. Carries normalized detection results
 */
class ImgDetections : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgDetections& dets;

   public:
    /// Construct ImgDetections message
    ImgDetections();
    explicit ImgDetections(std::shared_ptr<RawImgDetections> ptr);
    virtual ~ImgDetections() = default;

    /// Detections
    std::vector<ImgDetection>& detections;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    ImgDetections& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    ImgDetections& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    ImgDetections& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
