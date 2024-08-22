#include "depthai/pipeline/node/EdgeDetector.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

EdgeDetector::EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : EdgeDetector(par, nodeId, std::make_unique<EdgeDetector::Properties>()) {}
EdgeDetector::EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, EdgeDetector, EdgeDetectorProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawEdgeDetectorConfig>()),
      initialConfig(rawConfig) {
    setInputRefs({&inputConfig, &inputImage});
    setOutputRefs({&outputImage, &passthroughInputImage});
}

EdgeDetector::Properties& EdgeDetector::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

// Node properties configuration
void EdgeDetector::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool EdgeDetector::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void EdgeDetector::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void EdgeDetector::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

}  // namespace node
}  // namespace dai
