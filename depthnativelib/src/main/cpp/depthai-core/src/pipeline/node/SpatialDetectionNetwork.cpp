#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------
SpatialDetectionNetwork::SpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : SpatialDetectionNetwork(par, nodeId, std::make_unique<Properties>()) {}
SpatialDetectionNetwork::SpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<DetectionNetwork, SpatialDetectionNetwork, SpatialDetectionNetworkProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&input, &inputDepth});
    setOutputRefs({&out, &boundingBoxMapping, &passthrough, &passthroughDepth});
}

void SpatialDetectionNetwork::setBoundingBoxScaleFactor(float scaleFactor) {
    properties.detectedBBScaleFactor = scaleFactor;
}

void SpatialDetectionNetwork::setDepthLowerThreshold(uint32_t lowerThreshold) {
    properties.depthThresholds.lowerThreshold = lowerThreshold;
}

void SpatialDetectionNetwork::setDepthUpperThreshold(uint32_t upperThreshold) {
    properties.depthThresholds.upperThreshold = upperThreshold;
}

void SpatialDetectionNetwork::setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm calculationAlgorithm) {
    properties.calculationAlgorithm = calculationAlgorithm;
}

void SpatialDetectionNetwork::setSpatialCalculationStepSize(int stepSize) {
    properties.stepSize = stepSize;
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
MobileNetSpatialDetectionNetwork::MobileNetSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NodeCRTP<SpatialDetectionNetwork, MobileNetSpatialDetectionNetwork, SpatialDetectionNetworkProperties>(par, nodeId) {
    properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
YoloSpatialDetectionNetwork::YoloSpatialDetectionNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : NodeCRTP<SpatialDetectionNetwork, YoloSpatialDetectionNetwork, SpatialDetectionNetworkProperties>(par, nodeId) {
    properties.parser.nnFamily = DetectionNetworkType::YOLO;
}

void YoloSpatialDetectionNetwork::setNumClasses(const int numClasses) {
    properties.parser.classes = numClasses;
}

void YoloSpatialDetectionNetwork::setCoordinateSize(const int coordinates) {
    properties.parser.coordinates = coordinates;
}

void YoloSpatialDetectionNetwork::setAnchors(std::vector<float> anchors) {
    properties.parser.anchors = anchors;
}

void YoloSpatialDetectionNetwork::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    properties.parser.anchorMasks = anchorMasks;
}

void YoloSpatialDetectionNetwork::setIouThreshold(float thresh) {
    properties.parser.iouThreshold = thresh;
}

/// Get num classes
int YoloSpatialDetectionNetwork::getNumClasses() const {
    return properties.parser.classes;
}

/// Get coordianate size
int YoloSpatialDetectionNetwork::getCoordinateSize() const {
    return properties.parser.coordinates;
}

/// Get anchors
std::vector<float> YoloSpatialDetectionNetwork::getAnchors() const {
    return properties.parser.anchors;
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloSpatialDetectionNetwork::getAnchorMasks() const {
    return properties.parser.anchorMasks;
}

/// Get Iou threshold
float YoloSpatialDetectionNetwork::getIouThreshold() const {
    return properties.parser.iouThreshold;
}

}  // namespace node
}  // namespace dai
