#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * StereoDepthConfig message.
 */
class StereoDepthConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawStereoDepthConfig& cfg;

   public:
    using MedianFilter = dai::MedianFilter;
    using AlgorithmControl = RawStereoDepthConfig::AlgorithmControl;
    using PostProcessing = RawStereoDepthConfig::PostProcessing;
    using CensusTransform = RawStereoDepthConfig::CensusTransform;
    using CostMatching = RawStereoDepthConfig::CostMatching;
    using CostAggregation = RawStereoDepthConfig::CostAggregation;

    /**
     * Construct StereoDepthConfig message.
     */
    StereoDepthConfig();
    explicit StereoDepthConfig(std::shared_ptr<RawStereoDepthConfig> ptr);
    virtual ~StereoDepthConfig() = default;

    /**
     * @param align Set the disparity/depth alignment: centered (between the 'left' and 'right' inputs),
     * or from the perspective of a rectified output stream
     */
    StereoDepthConfig& setDepthAlign(AlgorithmControl::DepthAlign align);

    /**
     * Confidence threshold for disparity calculation
     * @param confThr Confidence threshold value 0..255
     */
    StereoDepthConfig& setConfidenceThreshold(int confThr);
    /**
     * Get confidence threshold for disparity calculation
     */
    int getConfidenceThreshold() const;

    /**
     * @param median Set kernel size for disparity/depth median filtering, or disable
     */
    StereoDepthConfig& setMedianFilter(MedianFilter median);
    /**
     * Get median filter setting
     */
    MedianFilter getMedianFilter() const;

    /**
     * A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together,
     * resulting in larger areas of semi-equal color.
     * @param sigma Set sigma value for 5x5 bilateral filter. 0..65535
     */
    StereoDepthConfig& setBilateralFilterSigma(uint16_t sigma);
    /**
     * Get sigma value for 5x5 bilateral filter
     */
    uint16_t getBilateralFilterSigma() const;

    /**
     * @param threshold Set threshold for left-right, right-left disparity map combine, 0..255
     */
    StereoDepthConfig& setLeftRightCheckThreshold(int threshold);
    /**
     * Get threshold for left-right check combine
     */
    int getLeftRightCheckThreshold() const;

    /**
     * Computes and combines disparities in both L-R and R-L directions, and combine them.
     *
     * For better occlusion handling, discarding invalid disparity values
     */
    StereoDepthConfig& setLeftRightCheck(bool enable);

    /**
     * Disparity range increased from 95 to 190, combined from full resolution and downscaled images.
     * Suitable for short range objects
     */
    StereoDepthConfig& setExtendedDisparity(bool enable);

    /**
     * Computes disparity with sub-pixel interpolation (3 fractional bits by default).
     *
     * Suitable for long range. Currently incompatible with extended disparity
     */
    StereoDepthConfig& setSubpixel(bool enable);

    /**
     * Number of fractional bits for subpixel mode.
     * Default value: 3.
     * Valid values: 3,4,5.
     * Defines the number of fractional disparities: 2^x.
     * Median filter postprocessing is supported only for 3 fractional bits.
     */
    StereoDepthConfig& setSubpixelFractionalBits(int subpixelFractionalBits);

    /**
     * Set depth unit of depth map.
     *
     * Meter, centimeter, millimeter, inch, foot or custom unit is available.
     */
    StereoDepthConfig& setDepthUnit(AlgorithmControl::DepthUnit depthUnit);

    /**
     * Shift input frame by a number of pixels to increase minimum depth.
     * For example shifting by 48 will change effective disparity search range from (0,95] to [48,143].
     * An alternative approach to reducing the minZ.
     * We normally only recommend doing this when it is known that there will be no objects
     * farther away than MaxZ, such as having a depth camera mounted above a table
     * pointing down at the table surface.
     */
    StereoDepthConfig& setDisparityShift(int disparityShift);

    /**
     * Invalidate X amount of pixels at the edge of disparity frame.
     * For right and center alignment X pixels will be invalidated from the right edge,
     * for left alignment from the left edge.
     */
    StereoDepthConfig& setNumInvalidateEdgePixels(int32_t numInvalidateEdgePixels);

    /**
     * Get depth unit of depth map.
     */
    AlgorithmControl::DepthUnit getDepthUnit();

    /**
     * Useful for normalization of the disparity map.
     * @returns Maximum disparity value that the node can return
     */
    float getMaxDisparity() const;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    StereoDepthConfig& set(dai::RawStereoDepthConfig config);

    /**
     * Retrieve configuration data for StereoDepth.
     * @returns config for stereo depth algorithm
     */
    dai::RawStereoDepthConfig get() const;
};

}  // namespace dai
