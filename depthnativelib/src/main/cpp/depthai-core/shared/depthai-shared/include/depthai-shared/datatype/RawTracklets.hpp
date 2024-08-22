#pragma once

#include <iostream>

#include "depthai-shared/common/Point3f.hpp"
#include "depthai-shared/common/Rect.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawImgDetections.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Tracklet structure
 *
 * Contains tracklets from object tracker output.
 */
struct Tracklet {
    enum class TrackingStatus : std::int32_t {
        NEW,     /**< The object is newly added. */
        TRACKED, /**< The object is being tracked. */
        LOST,   /**< The object gets lost now. The object can be tracked again automatically(long term tracking) or by specifying detected object manually(short
                  term and zero term tracking). */
        REMOVED /**< The object is removed. */
    };
    /**
     * Tracked region of interest.
     */
    Rect roi;
    /**
     * Tracklet's ID.
     */
    std::int32_t id = 0;
    /**
     * Tracklet's label ID.
     */
    std::int32_t label = 0;
    /**
     * Number of frames it is being tracked for.
     */
    std::int32_t age = 0;
    /**
     * Status of tracklet.
     */
    TrackingStatus status = TrackingStatus::LOST;

    /**
     * Image detection that is tracked.
     */
    ImgDetection srcImgDetection;
    /**
     * Spatial coordinates of tracklet.
     */
    Point3f spatialCoordinates;
    DEPTHAI_SERIALIZE(Tracklet, roi, id, label, age, status, srcImgDetection, spatialCoordinates);
};

/// RawTracklets structure
struct RawTracklets : public RawBuffer {
    std::vector<Tracklet> tracklets;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Tracklets;
    };

    DEPTHAI_SERIALIZE(RawTracklets, tracklets, RawBuffer::sequenceNum, RawBuffer::ts, RawBuffer::tsDevice);
};

}  // namespace dai
