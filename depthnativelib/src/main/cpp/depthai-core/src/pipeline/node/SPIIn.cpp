#include "depthai/pipeline/node/SPIIn.hpp"

namespace dai {
namespace node {

SPIIn::SPIIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : SPIIn(par, nodeId, std::make_unique<SPIIn::Properties>()) {}
SPIIn::SPIIn(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, SPIIn, SPIInProperties>(par, nodeId, std::move(props)) {
    properties.busId = 0;
    setOutputRefs({&out});
}

void SPIIn::setStreamName(const std::string& name) {
    properties.streamName = name;
}

void SPIIn::setBusId(int busId) {
    properties.busId = busId;
}

void SPIIn::setMaxDataSize(std::uint32_t maxDataSize) {
    properties.maxDataSize = maxDataSize;
}

void SPIIn::setNumFrames(std::uint32_t numFrames) {
    properties.numFrames = numFrames;
}

std::string SPIIn::getStreamName() const {
    return properties.streamName;
}

int SPIIn::getBusId() const {
    return properties.busId;
}

std::uint32_t SPIIn::getMaxDataSize() const {
    return properties.maxDataSize;
}

std::uint32_t SPIIn::getNumFrames() const {
    return properties.numFrames;
}

}  // namespace node
}  // namespace dai
