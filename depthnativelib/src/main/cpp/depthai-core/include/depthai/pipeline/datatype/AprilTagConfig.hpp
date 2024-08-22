#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTagConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTagConfig message.
 */
class AprilTagConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAprilTagConfig& cfg;

   public:
    using Family = RawAprilTagConfig::Family;
    using QuadThresholds = RawAprilTagConfig::QuadThresholds;

    /**
     * Construct AprilTagConfig message.
     */
    AprilTagConfig();
    explicit AprilTagConfig(std::shared_ptr<RawAprilTagConfig> ptr);
    virtual ~AprilTagConfig() = default;

    /**
     * @param family AprilTag family
     */
    AprilTagConfig& setFamily(Family family);

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    AprilTagConfig& set(dai::RawAprilTagConfig config);

    /**
     * Retrieve configuration data for AprilTag.
     * @returns config for stereo depth algorithm
     */
    dai::RawAprilTagConfig get() const;
};

}  // namespace dai
