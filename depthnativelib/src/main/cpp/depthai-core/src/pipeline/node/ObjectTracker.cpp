#include "depthai/pipeline/node/ObjectTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ObjectTracker::ObjectTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : ObjectTracker(par, nodeId, std::make_unique<ObjectTracker::Properties>()) {}
ObjectTracker::ObjectTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, ObjectTracker, ObjectTrackerProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&inputTrackerFrame, &inputDetectionFrame, &inputDetections});
    setOutputRefs({&out, &passthroughTrackerFrame, &passthroughDetectionFrame, &passthroughDetections});
}

void ObjectTracker::setTrackerThreshold(float threshold) {
    properties.trackerThreshold = threshold;
}

void ObjectTracker::setMaxObjectsToTrack(std::int32_t maxObjectsToTrack) {
    properties.maxObjectsToTrack = maxObjectsToTrack;
}

void ObjectTracker::setDetectionLabelsToTrack(std::vector<std::uint32_t> labels) {
    properties.detectionLabelsToTrack = labels;
}

void ObjectTracker::setTrackerType(TrackerType type) {
    properties.trackerType = type;
}

void ObjectTracker::setTrackerIdAssignmentPolicy(TrackerIdAssignmentPolicy type) {
    properties.trackerIdAssignmentPolicy = type;
}
void ObjectTracker::setTrackingPerClass(bool trackingPerClass) {
    properties.trackingPerClass = trackingPerClass;
}

}  // namespace node
}  // namespace dai
