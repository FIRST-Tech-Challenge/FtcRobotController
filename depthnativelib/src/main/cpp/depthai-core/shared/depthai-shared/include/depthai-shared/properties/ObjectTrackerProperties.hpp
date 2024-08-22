#pragma once

// std
#include <vector>

// project
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

enum class TrackerType : std::int32_t {
    /// Kernelized Correlation Filter tracking
    SHORT_TERM_KCF = 1,
    /// Short term tracking without using image data
    SHORT_TERM_IMAGELESS = 3,
    /// Ability to track the objects without accessing image data.
    ZERO_TERM_IMAGELESS = 5,
    /// Tracking using image data too.
    ZERO_TERM_COLOR_HISTOGRAM = 6
};

enum class TrackerIdAssignmentPolicy : std::int32_t {
    /// Always take a new, unique ID
    UNIQUE_ID,
    /// Take the smallest available ID
    SMALLEST_ID
};

/**
 * Specify properties for ObjectTracker
 */
struct ObjectTrackerProperties : PropertiesSerializable<Properties, ObjectTrackerProperties> {
    /**
     * Confidence threshold for tracklets.
     * Above this threshold detections will be tracked.
     * Default 0, all detections are tracked.
     */
    float trackerThreshold = 0.0;
    /**
     * Maximum number of objects to track.
     * Maximum 60 for SHORT_TERM_KCF, maximum 1000 for other tracking methods.
     * Default 60.
     */
    std::int32_t maxObjectsToTrack = 60;
    /**
     * Which detections labels to track.
     * Default all labels are tracked.
     */
    std::vector<std::uint32_t> detectionLabelsToTrack;
    /**
     * Tracking method.
     */
    TrackerType trackerType = TrackerType::ZERO_TERM_IMAGELESS;
    /**
     * New ID assignment policy.
     */
    TrackerIdAssignmentPolicy trackerIdAssignmentPolicy = TrackerIdAssignmentPolicy::UNIQUE_ID;
    /**
     * Whether tracker should take into consideration class label for tracking.
     */
    bool trackingPerClass = true;
};

DEPTHAI_SERIALIZE_EXT(
    ObjectTrackerProperties, trackerThreshold, maxObjectsToTrack, detectionLabelsToTrack, trackerType, trackerIdAssignmentPolicy, trackingPerClass);

}  // namespace dai
