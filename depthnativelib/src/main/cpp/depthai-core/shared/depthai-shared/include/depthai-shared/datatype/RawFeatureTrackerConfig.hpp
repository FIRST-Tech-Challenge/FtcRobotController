#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawFeatureTrackerConfig configuration structure
struct RawFeatureTrackerConfig : public RawBuffer {
    static constexpr const std::int32_t AUTO = 0;

    /**
     * Corner detector configuration structure.
     */
    struct CornerDetector {
        enum class Type : std::int32_t {
            /**
             * Harris corner detector.
             */
            HARRIS,
            /**
             * Shi-Thomasi corner detector.
             */
            SHI_THOMASI
        };
        /**
         * Corner detector algorithm type.
         */
        Type type = Type::HARRIS;

        /**
         * Ensures distributed feature detection across the image.
         * Image is divided into horizontal and vertical cells,
         * each cell has a target feature count = numTargetFeatures / cellGridDimension.
         * Each cell has its own feature threshold.
         * A value of 4 means that the image is divided into 4x4 cells of equal width/height.
         * Maximum 4, minimum 1.
         */
        std::int32_t cellGridDimension = 4;

        /**
         * Target number of features to detect.
         * Maximum number of features is determined at runtime based on algorithm type.
         */
        std::int32_t numTargetFeatures = 320;

        /**
         * Hard limit for the maximum number of features that can be detected.
         * 0 means auto, will be set to the maximum value based on memory constraints.
         */
        std::int32_t numMaxFeatures = AUTO;

        /**
         * Enable 3x3 Sobel operator to smoothen the image whose gradient is to be computed.
         * If disabled, a simple 1D row/column differentiator is used for gradient.
         */
        bool enableSobel = true;

        /**
         * Enable sorting detected features based on their score or not.
         */
        bool enableSorting = true;

        /**
         * Threshold settings structure for corner detector.
         */
        struct Thresholds {
            /**
             * Minimum strength of a feature which will be detected.
             * 0 means automatic threshold update. Recommended so the tracker can adapt to different scenes/textures.
             * Each cell has its own threshold.
             * Empirical value.
             */
            float initialValue = AUTO;

            /**
             * Minimum limit for threshold.
             * Applicable when automatic threshold update is enabled.
             * 0 means auto, 6000000 for HARRIS, 1200 for SHI_THOMASI.
             * Empirical value.
             */
            float min = AUTO;

            /**
             * Maximum limit for threshold.
             * Applicable when automatic threshold update is enabled.
             * 0 means auto.
             * Empirical value.
             */
            float max = AUTO;

            /**
             * When detected number of features exceeds the maximum in a cell threshold is lowered
             * by multiplying its value with this factor.
             */
            float decreaseFactor = 0.9f;

            /**
             * When detected number of features doesn't exceed the maximum in a cell, threshold is increased
             * by multiplying its value with this factor.
             */
            float increaseFactor = 1.1f;
            DEPTHAI_SERIALIZE(Thresholds, initialValue, min, max, decreaseFactor, increaseFactor);
        };

        /**
         * Threshold settings.
         * These are advanced settings, suitable for debugging/special cases.
         */
        Thresholds thresholds;

        DEPTHAI_SERIALIZE(CornerDetector, type, cellGridDimension, numTargetFeatures, numMaxFeatures, thresholds, enableSobel, enableSorting);
    };

    /**
     * Used for feature reidentification between current and previous features.
     */
    struct MotionEstimator {
        /**
         * Enable motion estimation or not.
         */
        bool enable = true;

        enum class Type : std::int32_t {
            /**
             * Using the pyramidal Lucas-Kanade optical flow method.
             */
            LUCAS_KANADE_OPTICAL_FLOW,
            /**
             * Using a dense motion estimation hardware block (Block matcher).
             */
            HW_MOTION_ESTIMATION
        };
        /**
         * Motion estimator algorithm type.
         */
        Type type = Type::LUCAS_KANADE_OPTICAL_FLOW;

        /**
         * Optical flow configuration structure.
         */
        struct OpticalFlow {
            /**
             * Number of pyramid levels, only for optical flow.
             * AUTO means it's decided based on input resolution: 3 if image width <= 640, else 4.
             * Valid values are either 3/4 for VGA, 4 for 720p and above.
             */
            std::int32_t pyramidLevels = AUTO;

            /**
             * Image patch width used to track features.
             * Must be an odd number, maximum 9.
             * N means the algorithm will be able to track motion at most (N-1)/2 pixels in a direction per pyramid level.
             * Increasing this number increases runtime
             */
            std::int32_t searchWindowWidth = 5;
            /**
             * Image patch height used to track features.
             * Must be an odd number, maximum 9.
             * N means the algorithm will be able to track motion at most (N-1)/2 pixels in a direction per pyramid level.
             * Increasing this number increases runtime
             */
            std::int32_t searchWindowHeight = 5;

            /**
             * Feature tracking termination criteria.
             * Optical flow will refine the feature position on each pyramid level until
             * the displacement between two refinements is smaller than this value.
             * Decreasing this number increases runtime.
             */
            float epsilon = 0.01f;

            /**
             * Feature tracking termination criteria. Optical flow will refine the feature position maximum this many times
             * on each pyramid level. If the Epsilon criteria described in the previous chapter is not met after this number
             * of iterations, the algorithm will continue with the current calculated value.
             * Increasing this number increases runtime.
             */
            std::int32_t maxIterations = 9;

            DEPTHAI_SERIALIZE(OpticalFlow, pyramidLevels, searchWindowWidth, searchWindowHeight, epsilon, maxIterations);
        };

        /**
         * Optical flow configuration.
         * Takes effect only if MotionEstimator algorithm type set to LUCAS_KANADE_OPTICAL_FLOW.
         */
        OpticalFlow opticalFlow;

        DEPTHAI_SERIALIZE(MotionEstimator, enable, type, opticalFlow);
    };

    /**
     * FeatureMaintainer configuration structure.
     */
    struct FeatureMaintainer {
        /**
         * Enable feature maintaining or not.
         */
        bool enable = true;

        /**
         * Used to filter out detected feature points that are too close.
         * Requires sorting enabled in detector.
         * Unit of measurement is squared euclidean distance in pixels.
         */
        float minimumDistanceBetweenFeatures = 50;

        /**
         * Optical flow measures the tracking error for every feature.
         * If the point can’t be tracked or it’s out of the image it will set this error to a maximum value.
         * This threshold defines the level where the tracking accuracy is considered too bad to keep the point.
         */
        float lostFeatureErrorThreshold = 50000;

        /**
         * Once a feature was detected and we started tracking it, we need to update its Harris score on each image.
         * This is needed because a feature point can disappear, or it can become too weak to be tracked. This
         * threshold defines the point where such a feature must be dropped.
         * As the goal of the algorithm is to provide longer tracks, we try to add strong points and track them until
         * they are absolutely untrackable. This is why, this value is usually smaller than the detection threshold.
         */
        float trackedFeatureThreshold = 200000;

        DEPTHAI_SERIALIZE(FeatureMaintainer, enable, minimumDistanceBetweenFeatures, lostFeatureErrorThreshold, trackedFeatureThreshold);
    };

    /**
     * Corner detector configuration.
     * Used for feature detection.
     */
    CornerDetector cornerDetector;

    /**
     * Motion estimator configuration.
     * Used for feature reidentification between current and previous features.
     */
    MotionEstimator motionEstimator;

    /**
     * FeatureMaintainer configuration.
     * Used for feature maintaining.
     */
    FeatureMaintainer featureMaintainer;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::FeatureTrackerConfig;
    };

    DEPTHAI_SERIALIZE(RawFeatureTrackerConfig, cornerDetector, motionEstimator, featureMaintainer);
};

}  // namespace dai
