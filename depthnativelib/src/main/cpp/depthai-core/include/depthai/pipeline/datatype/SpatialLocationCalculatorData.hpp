#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialLocations.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * SpatialLocationCalculatorData message. Carries spatial information (X,Y,Z) and their configuration parameters
 */
class SpatialLocationCalculatorData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialLocations& rawdata;

   public:
    /**
     * Construct SpatialLocationCalculatorData message.
     */
    SpatialLocationCalculatorData();
    explicit SpatialLocationCalculatorData(std::shared_ptr<RawSpatialLocations> ptr);
    virtual ~SpatialLocationCalculatorData() = default;

    /**
     * Retrieve configuration data for SpatialLocationCalculatorData.
     * @returns Vector of spatial location data, carrying spatial information (X,Y,Z)
     */
    std::vector<SpatialLocations>& getSpatialLocations() const;

    std::vector<SpatialLocations>& spatialLocations;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    SpatialLocationCalculatorData& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    SpatialLocationCalculatorData& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    SpatialLocationCalculatorData& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
