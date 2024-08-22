#include "depthai/pipeline/node/AprilTag.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

AprilTag::AprilTag(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : AprilTag(par, nodeId, std::make_unique<AprilTag::Properties>()) {}
AprilTag::AprilTag(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, AprilTag, AprilTagProperties>(par, nodeId, std::move(props)), rawConfig(std::make_shared<RawAprilTagConfig>()), initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputImage});
    setOutputRefs({&out, &passthroughInputImage});
}

AprilTag::Properties& AprilTag::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void AprilTag::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

}  // namespace node
}  // namespace dai
