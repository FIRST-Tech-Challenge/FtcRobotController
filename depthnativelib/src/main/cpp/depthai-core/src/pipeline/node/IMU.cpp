#include "depthai/pipeline/node/IMU.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

IMU::IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : IMU(par, nodeId, std::make_unique<IMU::Properties>()) {}
IMU::IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, IMU, IMUProperties>(par, nodeId, std::move(props)) {
    setOutputRefs({&out});
}

void IMU::enableIMUSensor(IMUSensorConfig sensorConfig) {
    properties.imuSensors.push_back(sensorConfig);
}

void IMU::enableIMUSensor(const std::vector<IMUSensorConfig>& sensorConfigs) {
    properties.imuSensors = sensorConfigs;
}

void IMU::enableIMUSensor(IMUSensor sensor, uint32_t reportRate) {
    IMUSensorConfig config;
    config.sensorId = sensor;
    config.reportRate = reportRate;
    properties.imuSensors.push_back(config);
}

void IMU::enableIMUSensor(const std::vector<IMUSensor>& sensors, uint32_t reportRate) {
    std::vector<IMUSensorConfig> configs;
    for(auto& sensor : sensors) {
        IMUSensorConfig config;
        config.sensorId = sensor;
        config.reportRate = reportRate;
        configs.push_back(config);
    }
    properties.imuSensors = configs;
}

void IMU::setBatchReportThreshold(std::int32_t batchReportThreshold) {
    properties.batchReportThreshold = batchReportThreshold;
}

std::int32_t IMU::getBatchReportThreshold() const {
    return properties.batchReportThreshold;
}

void IMU::setMaxBatchReports(std::int32_t maxBatchReports) {
    properties.maxBatchReports = maxBatchReports;
}

std::int32_t IMU::getMaxBatchReports() const {
    return properties.maxBatchReports;
}

void IMU::enableFirmwareUpdate(bool enable) {
    properties.enableFirmwareUpdate = enable;
}

}  // namespace node
}  // namespace dai
