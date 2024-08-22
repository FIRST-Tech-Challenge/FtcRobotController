#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImageManipConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {

/**
 * ImageManipConfig message. Specifies image manipulation options like:
 *
 *  - Crop
 *
 *  - Resize
 *
 *  - Warp
 *
 *  - ...
 */
class ImageManipConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImageManipConfig& cfg;

   public:
    // Alias
    using CropConfig = RawImageManipConfig::CropConfig;
    using ResizeConfig = RawImageManipConfig::ResizeConfig;
    using FormatConfig = RawImageManipConfig::FormatConfig;

    /// Construct ImageManipConfig message
    ImageManipConfig();
    explicit ImageManipConfig(std::shared_ptr<RawImageManipConfig> ptr);
    virtual ~ImageManipConfig() = default;

    // Functions to set properties
    /**
     * Specifies crop with rectangle with normalized values (0..1)
     * @param xmin Top left X coordinate of rectangle
     * @param ymin Top left Y coordinate of rectangle
     * @param xmax Bottom right X coordinate of rectangle
     * @param ymax Bottom right Y coordinate of rectangle
     */
    ImageManipConfig& setCropRect(float xmin, float ymin, float xmax, float ymax);

    /**
     * Specifies crop with rectangle with normalized values (0..1)
     * @param coordinates Coordinate of rectangle
     */
    ImageManipConfig& setCropRect(std::tuple<float, float, float, float> coordinates);

    /**
     * Specifies crop with rotated rectangle. Optionally as non normalized coordinates
     * @param rr Rotated rectangle which specifies crop
     * @param normalizedCoords If true coordinates are in normalized range (0..1) otherwise absolute
     */
    ImageManipConfig& setCropRotatedRect(RotatedRect rr, bool normalizedCoords = true);

    /**
     * Specifies a centered crop.
     * @param ratio Ratio between input image and crop region (0..1)
     * @param whRatio Crop region aspect ratio - 1 equals to square, 1.7 equals to 16:9, ...
     */
    ImageManipConfig& setCenterCrop(float ratio, float whRatio = 1.0f);

    /**
     * Specifies warp by supplying 4 points in either absolute or normalized coordinates
     * @param pt 4 points specifying warp
     * @param normalizedCoords If true pt is interpreted as normalized, absolute otherwise
     */
    ImageManipConfig& setWarpTransformFourPoints(std::vector<Point2f> pt, bool normalizedCoords);

    /**
     * Specifies warp with a 3x3 matrix
     * @param mat 3x3 matrix
     */
    ImageManipConfig& setWarpTransformMatrix3x3(std::vector<float> mat);

    /**
     * Specifies that warp replicates border pixels
     */
    ImageManipConfig& setWarpBorderReplicatePixels();

    /**
     * Specifies fill color for border pixels. Example:
     *
     *  - setWarpBorderFillColor(255,255,255) -> white
     *
     *  - setWarpBorderFillColor(0,0,255) -> blue
     *
     * @param red Red component
     * @param green Green component
     * @param blue Blue component
     */
    ImageManipConfig& setWarpBorderFillColor(int red, int green, int blue);

    /**
     * Specifies clockwise rotation in degrees
     * @param deg Rotation in degrees
     */
    ImageManipConfig& setRotationDegrees(float deg);

    /**
     * Specifies clockwise rotation in radians
     * @param rad Rotation in radians
     */
    ImageManipConfig& setRotationRadians(float rad);

    /**
     * Specifies output image size. After crop stage the image will be stretched to fit.
     * @param w Width in pixels
     * @param h Height in pixels
     */
    ImageManipConfig& setResize(int w, int h);

    /**
     * Specifies output image size. After crop stage the image will be stretched to fit.
     * @param size Size in pixels
     */
    ImageManipConfig& setResize(std::tuple<int, int> size);

    /**
     * Specifies output image size. After crop stage the image will be resized by preserving aspect ration.
     * Optionally background can be specified.
     *
     * @param w Width in pixels
     * @param h Height in pixels
     * @param bgRed Red component
     * @param bgGreen Green component
     * @param bgBlue Blue component
     */
    ImageManipConfig& setResizeThumbnail(int w, int h, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);

    /**
     * Specifies output image size. After crop stage the image will be resized by preserving aspect ration.
     * Optionally background can be specified.
     *
     * @param size Size in pixels
     * @param bgRed Red component
     * @param bgGreen Green component
     * @param bgBlue Blue component
     */
    ImageManipConfig& setResizeThumbnail(std::tuple<int, int> size, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);

    /**
     * Specify output frame type.
     * @param name Frame type
     */
    ImageManipConfig& setFrameType(ImgFrame::Type name);

    /**
     * Specify gray to color conversion map
     * @param colormap map from Colormap enum or Colormap::NONE to disable
     */
    ImageManipConfig& setColormap(Colormap colormap, int min, int max);
    ImageManipConfig& setColormap(Colormap colormap, float maxf);
    ImageManipConfig& setColormap(Colormap colormap, int max = 255);

    /**
     * Specify horizontal flip
     * @param flip True to enable flip, false otherwise
     */
    ImageManipConfig& setHorizontalFlip(bool flip);

    /**
     * Specify vertical flip
     * @param flip True to enable vertical flip, false otherwise
     */
    void setVerticalFlip(bool flip);

    /**
     * Instruct ImageManip to not remove current image from its queue and use the same for next message.
     * @param reuse True to enable reuse, false otherwise
     */
    ImageManipConfig& setReusePreviousImage(bool reuse);

    /**
     * Instructs ImageManip to skip current image and wait for next in queue.
     * @param skip True to skip current image, false otherwise
     */
    ImageManipConfig& setSkipCurrentImage(bool skip);

    /**
     * Specifies to whether to keep aspect ratio or not
     */
    ImageManipConfig& setKeepAspectRatio(bool keep);

    /**
     * Specify which interpolation method to use
     * @param interpolation type of interpolation
     */
    ImageManipConfig& setInterpolation(dai::Interpolation interpolation);

    // Functions to retrieve properties
    /**
     * @returns Top left X coordinate of crop region
     */
    float getCropXMin() const;

    /**
     * @returns Top left Y coordinate of crop region
     */
    float getCropYMin() const;

    /**
     * @returns Bottom right X coordinate of crop region
     */
    float getCropXMax() const;

    /**
     * @returns Bottom right Y coordinate of crop region
     */
    float getCropYMax() const;

    /**
     * @returns Output image width
     */
    int getResizeWidth() const;

    /**
     * @returns Output image height
     */
    int getResizeHeight() const;

    /**
     * @returns Crop configuration
     */
    CropConfig getCropConfig() const;

    /**
     * @returns Resize configuration
     */
    ResizeConfig getResizeConfig() const;

    /**
     * @returns Format configuration
     */
    FormatConfig getFormatConfig() const;

    /**
     * @returns True if resize thumbnail mode is set, false otherwise
     */
    bool isResizeThumbnail() const;

    /**
     * @returns specified colormap
     */
    Colormap getColormap() const;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    ImageManipConfig& set(dai::RawImageManipConfig config);

    /**
     * Retrieve configuration data for ImageManip.
     * @returns config for ImageManip
     */
    dai::RawImageManipConfig get() const;

    /// Retrieve which interpolation method to use
    dai::Interpolation getInterpolation() const;
};

}  // namespace dai
