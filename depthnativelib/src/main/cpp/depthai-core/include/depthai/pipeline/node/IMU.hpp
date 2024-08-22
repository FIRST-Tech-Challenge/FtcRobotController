#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/IMUProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief IMU node for BNO08X.
 */
class IMU : public NodeCRTP<Node, IMU, IMUProperties> {
   public:
    constexpr static const char* NAME = "IMU";

    /**
     * Constructs IMU node.
     */
    IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Outputs IMUData message that carries IMU packets.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::IMUData, false}}};

    /**
     * Enable a new IMU sensor with explicit configuration
     */
    void enableIMUSensor(IMUSensorConfig sensorConfig);

    /**
     * Enable a list of IMU sensors with explicit configuration
     */
    void enableIMUSensor(const std::vector<IMUSensorConfig>& sensorConfigs);

    /**
     * Enable a new IMU sensor with default configuration
     */
    void enableIMUSensor(IMUSensor sensor, uint32_t reportRate);

    /**
     * Enable a list of IMU sensors with default configuration
     */
    void enableIMUSensor(const std::vector<IMUSensor>& sensors, uint32_t reportRate);

    /**
     * Above this packet threshold data will be sent to host, if queue is not blocked
     */
    void setBatchReportThreshold(std::int32_t batchReportThreshold);

    /**
     * Above this packet threshold data will be sent to host, if queue is not blocked
     */
    std::int32_t getBatchReportThreshold() const;

    /**
     * Maximum number of IMU packets in a batch report
     */
    void setMaxBatchReports(std::int32_t maxBatchReports);

    /**
     * Maximum number of IMU packets in a batch report
     */
    std::int32_t getMaxBatchReports() const;

    /*
     * Whether to perform firmware update or not.
     * Default value: false.
     */
    void enableFirmwareUpdate(bool enable);
};

}  // namespace node
}  // namespace dai
