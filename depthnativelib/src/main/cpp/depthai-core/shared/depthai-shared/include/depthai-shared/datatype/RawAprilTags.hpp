#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawAprilTagConfig.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * AprilTag structure.
 */
struct AprilTag {
    /**
     * The decoded ID of the tag
     */
    int id = 0;

    /**
     * How many error bits were corrected? Note: accepting large numbers of
     * corrected errors leads to greatly increased false positive rates.
     * As of this implementation, the detector cannot detect tags with
     * a hamming distance greater than 2.
     */
    int hamming = 0;

    /**
     * A measure of the quality of the binary decoding process; the
     * average difference between the intensity of a data bit versus
     * the decision threshold. Higher numbers roughly indicate better
     * decodes. This is a reasonable measure of detection accuracy
     * only for very small tags-- not effective for larger tags (where
     * we could have sampled anywhere within a bit cell and still
     * gotten a good detection.
     */
    float decisionMargin = 0.f;

    /**
     * The detected top left coordinates.
     */
    Point2f topLeft;

    /**
     * The detected top right coordinates.
     */
    Point2f topRight;

    /**
     * The detected bottom right coordinates.
     */
    Point2f bottomRight;

    /**
     * The detected bottom left coordinates.
     */
    Point2f bottomLeft;
};
DEPTHAI_SERIALIZE_EXT(AprilTag, id, hamming, decisionMargin, topLeft, topRight, bottomRight, bottomLeft);

/// RawAprilTags structure
struct RawAprilTags : public RawBuffer {
    std::vector<AprilTag> aprilTags;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::AprilTags;
    };

    DEPTHAI_SERIALIZE(RawAprilTags, aprilTags, RawBuffer::sequenceNum, RawBuffer::ts, RawBuffer::tsDevice);
};

}  // namespace dai
