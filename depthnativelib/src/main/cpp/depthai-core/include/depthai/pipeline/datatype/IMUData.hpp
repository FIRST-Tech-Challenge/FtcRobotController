#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawIMUData.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * IMUData message. Carries normalized detection results
 */
class IMUData : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawIMUData& rawIMU;

   public:
    /// Construct IMUData message
    IMUData();
    explicit IMUData(std::shared_ptr<RawIMUData> ptr);
    virtual ~IMUData() = default;

    /// Detections
    std::vector<IMUPacket>& packets;
};

}  // namespace dai
