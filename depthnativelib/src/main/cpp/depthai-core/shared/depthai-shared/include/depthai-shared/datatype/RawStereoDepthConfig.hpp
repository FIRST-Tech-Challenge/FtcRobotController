#pragma once
#include <cstdint>
#include <depthai-shared/common/optional.hpp>
#include <vector>

#include "depthai-shared/common/MedianFilter.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawStereoDepthConfig configuration structure
struct RawStereoDepthConfig : public RawBuffer {
    using MedianFilter = dai::MedianFilter;

    struct AlgorithmControl {
        /**
         * Align the disparity/depth to the perspective of a rectified output, or center it
         */
        enum class DepthAlign : int32_t { RECTIFIED_RIGHT, RECTIFIED_LEFT, CENTER };

        /**
         * Measurement unit for depth data
         */
        enum class DepthUnit : int32_t { METER, CENTIMETER, MILLIMETER, INCH, FOOT, CUSTOM };

        /**
         * Set the disparity/depth alignment to the perspective of a rectified output, or center it
         */
        DepthAlign depthAlign = DepthAlign::RECTIFIED_RIGHT;

        /**
         * Measurement unit for depth data.
         * Depth data is integer value, multiple of depth unit.
         */
        DepthUnit depthUnit = DepthUnit::MILLIMETER;

        /**
         * Custom depth unit multiplier, if custom depth unit is enabled, relative to 1 meter.
         * A multiplier of 1000 effectively means depth unit in millimeter.
         */
        float customDepthUnitMultiplier = 1000.f;

        /**
         * Computes and combines disparities in both L-R and R-L directions, and combine them.
         * For better occlusion handling
         */
        bool enableLeftRightCheck = true;

        /**
         * Disparity range increased from 95 to 190, combined from full resolution and downscaled images.
         * Suitable for short range objects
         */
        bool enableExtended = false;

        /**
         * Computes disparity with sub-pixel interpolation (5 fractional bits), suitable for long range
         */
        bool enableSubpixel = false;

        /**
         * Left-right check threshold for left-right, right-left disparity map combine, 0..128
         * Used only when left-right check mode is enabled.
         * Defines the maximum difference between the confidence of pixels from left-right and right-left confidence maps
         */
        std::int32_t leftRightCheckThreshold = 10;

        /**
         * Number of fractional bits for subpixel mode
         *
         * Valid values: 3,4,5
         *
         * Defines the number of fractional disparities: 2^x
         *
         * Median filter postprocessing is supported only for 3 fractional bits
         */
        std::int32_t subpixelFractionalBits = 3;

        /**
         * Shift input frame by a number of pixels to increase minimum depth.
         * For example shifting by 48 will change effective disparity search range from (0,95] to [48,143].
         * An alternative approach to reducing the minZ.
         * We normally only recommend doing this when it is known that there will be no objects
         * farther away than MaxZ, such as having a depth camera mounted above a table
         * pointing down at the table surface.
         */
        std::int32_t disparityShift = 0;

        /**
         * Used only for debug purposes. centerAlignmentShiftFactor is set automatically in firmware,
         * from camera extrinsics when depth alignment to camera is enabled.
         * Center alignment is achieved by shifting the obtained disparity map by a scale factor.
         * It's used to align to a different camera that is on the same horizontal baseline as the two stereo cameras.
         * E.g. if we have a device with 10 cm stereo baseline, and we have another camera inbetween,
         * 9cm from the LEFT camera and 1 cm from the RIGHT camera we can align the obtained disparity map using a scale factor of 0.9.
         * Note that aligning disparity map to a different camera involves 2 steps:
         * 1. Shifting obtained disparity map.
         * 2. Warping the image to counter rotate and scaling to match the FOV.
         * Center alignment factor 1 is equivalent to RECTIFIED_RIGHT
         * Center alignment factor 0 is equivalent to RECTIFIED_LEFT
         */
        tl::optional<float> centerAlignmentShiftFactor;

        /**
         * Invalidate X amount of pixels at the edge of disparity frame.
         * For right and center alignment X pixels will be invalidated from the right edge,
         * for left alignment from the left edge.
         */
        std::int32_t numInvalidateEdgePixels = 0;

        DEPTHAI_SERIALIZE(AlgorithmControl,
                          depthAlign,
                          depthUnit,
                          customDepthUnitMultiplier,
                          enableLeftRightCheck,
                          enableExtended,
                          enableSubpixel,
                          leftRightCheckThreshold,
                          subpixelFractionalBits,
                          disparityShift,
                          centerAlignmentShiftFactor,
                          numInvalidateEdgePixels);
    };

    /**
     * Controls the flow of stereo algorithm - left-right check, subpixel etc.
     */
    AlgorithmControl algorithmControl;

    /**
     * Post-processing filters, all the filters are applied in disparity domain.
     */
    struct PostProcessing {
        /**
         * Set kernel size for disparity/depth median filtering, or disable
         */
        MedianFilter median = MedianFilter::KERNEL_5x5;

        /**
         * Sigma value for bilateral filter. 0 means disabled.
         * A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together.
         */
        std::int16_t bilateralSigmaValue = 0;

        /**
         * 1D edge-preserving spatial filter using high-order domain transform.
         */
        struct SpatialFilter {
            static constexpr const std::int32_t DEFAULT_DELTA_VALUE = 3;

            /**
             * Whether to enable or disable the filter.
             */
            bool enable = false;

            /**
             * An in-place heuristic symmetric hole-filling mode applied horizontally during the filter passes.
             * Intended to rectify minor artefacts with minimal performance impact.
             * Search radius for hole filling.
             */
            std::uint8_t holeFillingRadius = 2;

            /**
             * The Alpha factor in an exponential moving average with Alpha=1 - no filter. Alpha = 0 - infinite filter.
             * Determines the amount of smoothing.
             */
            float alpha = 0.5f;

            /**
             * Step-size boundary. Establishes the threshold used to preserve "edges".
             * If the disparity value between neighboring pixels exceed the disparity threshold set by this delta parameter,
             * then filtering will be temporarily disabled.
             * Default value 0 means auto: 3 disparity integer levels.
             * In case of subpixel mode it's 3*number of subpixel levels.
             */
            std::int32_t delta = 0;

            /**
             * Number of iterations over the image in both horizontal and vertical direction.
             */
            std::int32_t numIterations = 1;

            DEPTHAI_SERIALIZE(SpatialFilter, enable, holeFillingRadius, alpha, delta, numIterations);
        };

        /**
         * Edge-preserving filtering: This type of filter will smooth the depth noise while attempting to preserve edges.
         */
        SpatialFilter spatialFilter;

        /**
         * Temporal filtering with optional persistence.
         */
        struct TemporalFilter {
            static constexpr const std::int32_t DEFAULT_DELTA_VALUE = 3;

            /**
             * Whether to enable or disable the filter.
             */
            bool enable = false;

            /**
             * Persistency algorithm type.
             */
            enum class PersistencyMode : int32_t {
                PERSISTENCY_OFF = 0,
                VALID_8_OUT_OF_8 = 1,
                VALID_2_IN_LAST_3 = 2,
                VALID_2_IN_LAST_4 = 3,
                VALID_2_OUT_OF_8 = 4,
                VALID_1_IN_LAST_2 = 5,
                VALID_1_IN_LAST_5 = 6,
                VALID_1_IN_LAST_8 = 7,
                PERSISTENCY_INDEFINITELY = 8,
            };

            /**
             * Persistency mode.
             * If the current disparity/depth value is invalid, it will be replaced by an older value, based on persistency mode.
             */
            PersistencyMode persistencyMode = PersistencyMode::VALID_2_IN_LAST_4;

            /**
             * The Alpha factor in an exponential moving average with Alpha=1 - no filter. Alpha = 0 - infinite filter.
             * Determines the extent of the temporal history that should be averaged.
             */
            float alpha = 0.4f;

            /**
             * Step-size boundary. Establishes the threshold used to preserve surfaces (edges).
             * If the disparity value between neighboring pixels exceed the disparity threshold set by this delta parameter,
             * then filtering will be temporarily disabled.
             * Default value 0 means auto: 3 disparity integer levels.
             * In case of subpixel mode it's 3*number of subpixel levels.
             */
            std::int32_t delta = 0;

            DEPTHAI_SERIALIZE(TemporalFilter, enable, persistencyMode, alpha, delta);
        };

        /**
         * Temporal filtering with optional persistence.
         */
        TemporalFilter temporalFilter;

        /**
         * Threshold filtering.
         * Filters out distances outside of a given interval.
         */
        struct ThresholdFilter {
            /**
             * Minimum range in depth units.
             * Depth values under this value are invalidated.
             */
            std::int32_t minRange = 0;
            /**
             * Maximum range in depth units.
             * Depth values over this value are invalidated.
             */
            std::int32_t maxRange = 65535;

            DEPTHAI_SERIALIZE(ThresholdFilter, minRange, maxRange);
        };

        /**
         * Threshold filtering.
         * Filters out distances outside of a given interval.
         */
        ThresholdFilter thresholdFilter;

        /**
         * Brightness filtering.
         * If input frame pixel is too dark or too bright, disparity will be invalidated.
         * The idea is that for too dark/too bright pixels we have low confidence,
         * since that area was under/over exposed and details were lost.
         */
        struct BrightnessFilter {
            /**
             * Minimum pixel brightness.
             * If input pixel is less or equal than this value the depth value is invalidated.
             */
            std::int32_t minBrightness = 0;
            /**
             * Maximum range in depth units.
             * If input pixel is less or equal than this value the depth value is invalidated.
             */
            std::int32_t maxBrightness = 256;

            DEPTHAI_SERIALIZE(BrightnessFilter, minBrightness, maxBrightness);
        };

        /**
         * Brightness filtering.
         * If input frame pixel is too dark or too bright, disparity will be invalidated.
         * The idea is that for too dark/too bright pixels we have low confidence,
         * since that area was under/over exposed and details were lost.
         */
        BrightnessFilter brightnessFilter;

        /**
         * Speckle filtering.
         * Removes speckle noise.
         */
        struct SpeckleFilter {
            /**
             * Whether to enable or disable the filter.
             */
            bool enable = false;
            /**
             * Speckle search range.
             */
            std::uint32_t speckleRange = 50;

            DEPTHAI_SERIALIZE(SpeckleFilter, enable, speckleRange);
        };

        /**
         * Speckle filtering.
         * Removes speckle noise.
         */
        SpeckleFilter speckleFilter;

        /**
         * Decimation filter.
         * Reduces the depth scene complexity. The filter runs on kernel sizes [2x2] to [8x8] pixels.
         */
        struct DecimationFilter {
            /**
             * Decimation factor.
             * Valid values are 1,2,3,4.
             * Disparity/depth map x/y resolution will be decimated with this value.
             */
            std::uint32_t decimationFactor = 1;
            /**
             * Decimation algorithm type.
             */
            enum class DecimationMode : int32_t {
                PIXEL_SKIPPING = 0,
                NON_ZERO_MEDIAN = 1,
                NON_ZERO_MEAN = 2,
            };
            /**
             * Decimation algorithm type.
             */
            DecimationMode decimationMode = DecimationMode::PIXEL_SKIPPING;

            DEPTHAI_SERIALIZE(DecimationFilter, decimationFactor, decimationMode);
        };

        /**
         * Decimation filter.
         * Reduces disparity/depth map x/y complexity, reducing runtime complexity for other filters.
         */
        DecimationFilter decimationFilter;

        DEPTHAI_SERIALIZE(
            PostProcessing, median, bilateralSigmaValue, spatialFilter, temporalFilter, thresholdFilter, brightnessFilter, speckleFilter, decimationFilter);
    };

    /**
     * Controls the postprocessing of disparity and/or depth map.
     */
    PostProcessing postProcessing;

    /**
     * The basic cost function used by the Stereo Accelerator for matching the left and right images is the Census
     * Transform. It works on a block of pixels and computes a bit vector which represents the structure of the
     * image in that block.
     * There are two types of Census Transform based on how the middle pixel is used:
     * Classic Approach and Modified Census. The comparisons that are made between pixels can be or not thresholded.
     * In some cases a mask can be applied to filter out only specific bits from the entire bit stream.
     * All these approaches are:
     * Classic Approach: Uses middle pixel to compare against all its neighbors over a defined window. Each
     * comparison results in a new bit, that is 0 if central pixel is smaller, or 1 if is it bigger than its neighbor.
     * Modified Census Transform: same as classic Census Transform, but instead of comparing central pixel
     * with its neighbors, the window mean will be compared with each pixel over the window.
     * Thresholding Census Transform: same as classic Census Transform, but it is not enough that a
     * neighbor pixel to be bigger than the central pixel, it must be significant bigger (based on a threshold).
     * Census Transform with Mask: same as classic Census Transform, but in this case not all of the pixel from
     * the support window are part of the binary descriptor. We use a ma sk “M” to define which pixels are part
     * of the binary descriptor (1), and which pixels should be skipped (0).
     */
    struct CensusTransform {
        /**
         * Census transform kernel size possible values.
         */
        enum class KernelSize : std::int32_t { AUTO = -1, KERNEL_5x5 = 0, KERNEL_7x7, KERNEL_7x9 };

        /**
         * Census transform kernel size.
         */
        KernelSize kernelSize = KernelSize::AUTO;

        /**
         * Census transform mask, default - auto, mask is set based on resolution and kernel size.
         * Disabled for 400p input resolution.
         * Enabled for 720p.
         * 0XA82415 for 5x5 census transform kernel.
         * 0XAA02A8154055 for 7x7 census transform kernel.
         * 0X2AA00AA805540155 for 7x9 census transform kernel.
         * Empirical values.
         */
        uint64_t kernelMask = 0;

        /**
         * If enabled, each pixel in the window is compared with the mean window value instead of the central pixel.
         */
        bool enableMeanMode = true;

        /**
         * Census transform comparison threshold value.
         */
        uint32_t threshold = 0;

        DEPTHAI_SERIALIZE(CensusTransform, kernelSize, kernelMask, enableMeanMode, threshold);
    };

    /**
     * Census transform settings.
     */
    CensusTransform censusTransform;

    /**
     * The matching cost is way of measuring the similarity of image locations in stereo correspondence
     * algorithm. Based on the configuration parameters and based on the descriptor type, a linear equation
     * is applied to computing the cost for each candidate disparity at each pixel.
     */
    struct CostMatching {
        /**
         * Disparity search range: 64 or 96 pixels are supported by the HW.
         */
        enum class DisparityWidth : std::uint32_t { DISPARITY_64, DISPARITY_96 };

        /**
         * Disparity search range, default 96 pixels.
         */
        DisparityWidth disparityWidth = DisparityWidth::DISPARITY_96;

        /**
         * Disparity companding using sparse matching.
         * Matching pixel by pixel for N disparities.
         * Matching every 2nd pixel for M disparitites.
         * Matching every 4th pixel for T disparities.
         * In case of 96 disparities: N=48, M=32, T=16.
         * This way the search range is extended to 176 disparities, by sparse matching.
         * Note: when enabling this flag only depth map will be affected, disparity map is not.
         */
        bool enableCompanding = false;

        /**
         * Used only for debug purposes, SW postprocessing handled only invalid value of 0 properly.
         */
        uint8_t invalidDisparityValue = 0;

        /**
         * Disparities with confidence value under this threshold are accepted.
         * Higher confidence threshold means disparities with less confidence are accepted too.
         */
        uint8_t confidenceThreshold = 245;

        /**
         * The linear equation applied for computing the cost is:
         * COMB_COST = α*AD + β*(CTC<<3).
         * CLAMP(COMB_COST >> 5, threshold).
         * Where AD is the Absolute Difference between 2 pixels values.
         * CTC is the Census Transform Cost between 2 pixels, based on Hamming distance (xor).
         * The α and β parameters are subject to fine tuning by the user.
         */
        struct LinearEquationParameters {
            uint8_t alpha = 0;
            uint8_t beta = 2;
            uint8_t threshold = 127;

            DEPTHAI_SERIALIZE(LinearEquationParameters, alpha, beta, threshold);
        };

        /**
         * Cost calculation linear equation parameters.
         */
        LinearEquationParameters linearEquationParameters;

        DEPTHAI_SERIALIZE(CostMatching, disparityWidth, enableCompanding, invalidDisparityValue, confidenceThreshold, linearEquationParameters);
    };

    /**
     * Cost matching settings.
     */
    CostMatching costMatching;

    /**
     * Cost Aggregation is based on Semi Global Block Matching (SGBM). This algorithm uses a semi global
     * technique to aggregate the cost map. Ultimately the idea is to build inertia into the stereo algorithm. If
     * a pixel has very little texture information, then odds are the correct disparity for this pixel is close to
     * that of the previous pixel considered. This means that we get improved results in areas with low
     * texture.
     */
    struct CostAggregation {
        static constexpr const int defaultPenaltyP1 = 250;
        static constexpr const int defaultPenaltyP2 = 500;

        /**
         * Cost calculation linear equation parameters.
         */
        uint8_t divisionFactor = 1;

        /**
         * Horizontal P1 penalty cost parameter.
         */
        uint16_t horizontalPenaltyCostP1 = defaultPenaltyP1;
        /**
         * Horizontal P2 penalty cost parameter.
         */
        uint16_t horizontalPenaltyCostP2 = defaultPenaltyP2;

        /**
         * Vertical P1 penalty cost parameter.
         */
        uint16_t verticalPenaltyCostP1 = defaultPenaltyP1;
        /**
         * Vertical P2 penalty cost parameter.
         */
        uint16_t verticalPenaltyCostP2 = defaultPenaltyP2;

        DEPTHAI_SERIALIZE(CostAggregation, divisionFactor, horizontalPenaltyCostP1, horizontalPenaltyCostP2, verticalPenaltyCostP1, verticalPenaltyCostP2);
    };

    /**
     * Cost aggregation settings.
     */
    CostAggregation costAggregation;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::StereoDepthConfig;
    };

    DEPTHAI_SERIALIZE(RawStereoDepthConfig, algorithmControl, postProcessing, censusTransform, costMatching, costAggregation);
};

}  // namespace dai
