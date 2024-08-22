#include "depthai/pipeline/node/DetectionParser.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

DetectionParser::DetectionParser(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : DetectionParser(par, nodeId, std::make_unique<DetectionParser::Properties>()) {}
DetectionParser::DetectionParser(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, DetectionParser, DetectionParserProperties>(par, nodeId, std::move(props)), rawConfig(std::make_shared<RawEdgeDetectorConfig>()) {
    setInputRefs({&input});
    setOutputRefs({&out});
}

DetectionParser::Properties& DetectionParser::getProperties() {
    return properties;
}

void DetectionParser::setBlob(const OpenVINO::Blob& blob) {
    properties.networkInputs = blob.networkOutputs;
}

void DetectionParser::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

int DetectionParser::getNumFramesPool() {
    return properties.numFramesPool;
}

void DetectionParser::setNNFamily(DetectionNetworkType type) {
    properties.parser.nnFamily = type;
}

DetectionNetworkType DetectionParser::getNNFamily() {
    return properties.parser.nnFamily;
}

void DetectionParser::setConfidenceThreshold(float thresh) {
    properties.parser.confidenceThreshold = thresh;
}

float DetectionParser::getConfidenceThreshold() const {
    return properties.parser.confidenceThreshold;
}

void DetectionParser::setNumClasses(const int numClasses) {
    properties.parser.classes = numClasses;
}

void DetectionParser::setCoordinateSize(const int coordinates) {
    properties.parser.coordinates = coordinates;
}

void DetectionParser::setAnchors(std::vector<float> anchors) {
    properties.parser.anchors = anchors;
}

void DetectionParser::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.parser.anchorMasks = anchorMasks;
}

void DetectionParser::setIouThreshold(float thresh) {
    properties.parser.iouThreshold = thresh;
}

/// Get num classes
int DetectionParser::getNumClasses() const {
    return properties.parser.classes;
}

/// Get coordianate size
int DetectionParser::getCoordinateSize() const {
    return properties.parser.coordinates;
}

/// Get anchors
std::vector<float> DetectionParser::getAnchors() const {
    return properties.parser.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> DetectionParser::getAnchorMasks() const {
    return properties.parser.anchorMasks;
}

/// Get Iou threshold
float DetectionParser::getIouThreshold() const {
    return properties.parser.iouThreshold;
}

}  // namespace node
}  // namespace dai
