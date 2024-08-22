#pragma once

#include <cstdint>
#include <vector>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawAprilTags configuration structure
struct RawAprilTagConfig : public RawBuffer {
    /**
     * Supported AprilTag families.
     */
    enum class Family : std::int32_t { TAG_36H11 = 0, TAG_36H10, TAG_25H9, TAG_16H5, TAG_CIR21H7, TAG_STAND41H12 };

    /**
     * AprilTag family.
     */
    Family family = Family::TAG_36H11;

    /**
     * Detection of quads can be done on a lower-resolution image,
     * improving speed at a cost of pose accuracy and a slight
     * decrease in detection rate. Decoding the binary payload is
     * still done at full resolution.
     */
    std::int32_t quadDecimate = 4;

    /**
     * What Gaussian blur should be applied to the segmented image.
     * Parameter is the standard deviation in pixels.
     * Very noisy images benefit from non-zero values (e.g. 0.8).
     */
    float quadSigma = 0.0f;

    /**
     * When non-zero, the edges of the each quad are adjusted to "snap
     * to" strong gradients nearby. This is useful when decimation is
     * employed, as it can increase the quality of the initial quad
     * estimate substantially. Generally recommended to be on.
     * Very computationally inexpensive. Option is ignored if quadDecimate = 1.
     */
    bool refineEdges = true;

    /**
     * How much sharpening should be done to decoded images? This
     * can help decode small tags but may or may not help in odd
     * lighting conditions or low light conditions.
     * The default value is 0.25.
     */
    float decodeSharpening = 0.25f;

    /**
     * Max number of error bits that should be corrected. Accepting large numbers of
     * corrected errors leads to greatly increased false positive rates.
     * As of this implementation, the detector cannot detect tags with
     * a hamming distance greater than 2.
     */
    std::int32_t maxHammingDistance = 1;

    /**
     * AprilTag quad threshold parameters.
     */
    struct QuadThresholds {
        /**
         * Reject quads containing too few pixels.
         */
        std::int32_t minClusterPixels = 5;

        /**
         * How many corner candidates to consider when segmenting a group of pixels into a quad.
         */
        std::int32_t maxNmaxima = 10;

        /**
         * Reject quads where pairs of edges have angles that are close to
         * straight or close to 180 degrees. Zero means that no quads are
         * rejected. (In degrees).
         */
        float criticalDegree = 10.f;

        /**
         * When fitting lines to the contours, what is the maximum mean
         * squared error allowed?  This is useful in rejecting contours
         * that are far from being quad shaped; rejecting these quads "early"
         * saves expensive decoding processing.
         */
        float maxLineFitMse = 10.f;

        /**
         * When we build our model of black & white pixels, we add an
         * extra check that the white model must be (overall) brighter
         * than the black model. How much brighter? (in pixel values: [0,255]).
         */
        std::int32_t minWhiteBlackDiff = 5;

        /**
         * Should the thresholded image be deglitched? Only useful for very noisy images
         */
        bool deglitch = false;

        DEPTHAI_SERIALIZE(QuadThresholds, minClusterPixels, maxNmaxima, criticalDegree, maxLineFitMse, minWhiteBlackDiff, deglitch);
    };

    /**
     * AprilTag quad threshold parameters.
     */
    QuadThresholds quadThresholds;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::AprilTagConfig;
    };

    DEPTHAI_SERIALIZE(RawAprilTagConfig, family, quadDecimate, quadSigma, refineEdges, decodeSharpening, maxHammingDistance, quadThresholds);
};

}  // namespace dai
