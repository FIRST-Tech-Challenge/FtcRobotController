// First, as other headers may include <cmath>
#define _USE_MATH_DEFINES
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

#include <cmath>

namespace dai {

std::shared_ptr<RawBuffer> ImageManipConfig::serialize() const {
    return raw;
}

ImageManipConfig::ImageManipConfig() : Buffer(std::make_shared<RawImageManipConfig>()), cfg(*dynamic_cast<RawImageManipConfig*>(raw.get())) {}
ImageManipConfig::ImageManipConfig(std::shared_ptr<RawImageManipConfig> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawImageManipConfig*>(raw.get())) {}

// helpers
// Functions to set properties
ImageManipConfig& ImageManipConfig::setCropRect(float xmin, float ymin, float xmax, float ymax) {
    // Enable crop stage
    cfg.enableCrop = true;

    // Disable center crop
    cfg.cropConfig.enableCenterCropRectangle = false;

    // Set crop rect - limit to bounds beforehand
    cfg.cropConfig.cropRect.xmin = std::max(xmin, 0.0f);
    cfg.cropConfig.cropRect.ymin = std::max(ymin, 0.0f);
    cfg.cropConfig.cropRect.xmax = std::min(xmax, 1.0f);
    cfg.cropConfig.cropRect.ymax = std::min(ymax, 1.0f);
    return *this;
}

ImageManipConfig& ImageManipConfig::setCropRect(std::tuple<float, float, float, float> coordinates) {
    setCropRect(std::get<0>(coordinates), std::get<1>(coordinates), std::get<2>(coordinates), std::get<3>(coordinates));
    return *this;
}

ImageManipConfig& ImageManipConfig::setCropRotatedRect(RotatedRect rr, bool normalizedCoords) {
    // Enable crop stage and extended flags
    cfg.enableCrop = true;
    cfg.cropConfig.enableRotatedRect = true;

    cfg.cropConfig.cropRotatedRect = rr;
    cfg.cropConfig.normalizedCoords = normalizedCoords;
    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpTransformFourPoints(std::vector<Point2f> pt, bool normalizedCoords) {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.keepAspectRatio = false;
    cfg.resizeConfig.enableWarp4pt = true;
    cfg.resizeConfig.warpFourPoints = pt;
    cfg.resizeConfig.normalizedCoords = normalizedCoords;
    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpTransformMatrix3x3(std::vector<float> mat) {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.enableWarpMatrix = true;
    cfg.resizeConfig.warpMatrix3x3 = mat;
    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpBorderReplicatePixels() {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.warpBorderReplicate = true;
    return *this;
}

ImageManipConfig& ImageManipConfig::setWarpBorderFillColor(int red, int green, int blue) {
    // Enable resize stage and extended flags
    cfg.enableResize = true;
    cfg.resizeConfig.warpBorderReplicate = false;
    cfg.resizeConfig.bgRed = red;
    cfg.resizeConfig.bgGreen = green;
    cfg.resizeConfig.bgBlue = blue;
    return *this;
}

ImageManipConfig& ImageManipConfig::setCenterCrop(float ratio, float whRatio) {
    // Enable crop stage
    cfg.enableCrop = true;

    // Enable center center crop
    cfg.cropConfig.enableCenterCropRectangle = true;

    // Set crop center crop config
    cfg.cropConfig.cropRatio = ratio;
    // Limit to max 1.0f and disallow setting zero ratio
    if(ratio > 1.0f || ratio < 0.0f) {
        cfg.cropConfig.cropRatio = 1.0f;
    }

    cfg.cropConfig.widthHeightAspectRatio = whRatio;
    return *this;
}

ImageManipConfig& ImageManipConfig::setRotationDegrees(float deg) {
    cfg.enableResize = true;
    cfg.resizeConfig.rotationAngleDeg = deg;
    cfg.resizeConfig.enableRotation = true;
    return *this;
}

ImageManipConfig& ImageManipConfig::setRotationRadians(float rad) {
    static constexpr float rad2degFactor = static_cast<float>(180 / M_PI);
    setRotationDegrees(rad * rad2degFactor);
    return *this;
}

ImageManipConfig& ImageManipConfig::setResize(int w, int h) {
    // Enable resize stage
    cfg.enableResize = true;

    // Disable lock aspect ratio
    cfg.resizeConfig.lockAspectRatioFill = false;

    // Set resize config
    cfg.resizeConfig.width = w;
    cfg.resizeConfig.height = h;
    return *this;
}

ImageManipConfig& ImageManipConfig::setResize(std::tuple<int, int> size) {
    setResize(std::get<0>(size), std::get<1>(size));
    return *this;
}

ImageManipConfig& ImageManipConfig::setResizeThumbnail(int w, int h, int bgRed, int bgGreen, int bgBlue) {
    // Enable resize stage
    cfg.enableResize = true;

    // Set resize config
    cfg.resizeConfig.width = w;
    cfg.resizeConfig.height = h;

    // Set lock aspect ratio
    cfg.resizeConfig.lockAspectRatioFill = true;

    // Set background colors
    cfg.resizeConfig.bgRed = bgRed;
    cfg.resizeConfig.bgGreen = bgGreen;
    cfg.resizeConfig.bgBlue = bgBlue;
    return *this;
}

ImageManipConfig& ImageManipConfig::setResizeThumbnail(std::tuple<int, int> size, int bgRed, int bgGreen, int bgBlue) {
    setResizeThumbnail(std::get<0>(size), std::get<1>(size), bgRed, bgGreen, bgBlue);
    return *this;
}

ImageManipConfig& ImageManipConfig::setFrameType(dai::RawImgFrame::Type type) {
    // Enable format stage
    cfg.enableFormat = true;

    // Set type format
    cfg.formatConfig.type = type;
    return *this;
}

ImageManipConfig& ImageManipConfig::setColormap(dai::Colormap colormap, float maxf) {
    int max = maxf;
    if(max < 0 || max >= 256) throw std::invalid_argument("Colormap max argument must be between 0 and 255");

    // Enable format stage
    cfg.enableFormat = true;

    // Set type format
    cfg.formatConfig.colormap = colormap;
    cfg.formatConfig.colormapMin = 0;
    cfg.formatConfig.colormapMax = max;
    return *this;
}

ImageManipConfig& ImageManipConfig::setColormap(dai::Colormap colormap, int max) {
    if(max < 0 || max >= 256) throw std::invalid_argument("Colormap max argument must be between 0 and 255");

    // Enable format stage
    cfg.enableFormat = true;

    // Set type format
    cfg.formatConfig.colormap = colormap;
    cfg.formatConfig.colormapMin = 0;
    cfg.formatConfig.colormapMax = max;
    return *this;
}

ImageManipConfig& ImageManipConfig::setColormap(dai::Colormap colormap, int min, int max) {
    if(max < 0 || max >= 256) throw std::invalid_argument("Colormap max argument must be between 0 and 255");
    if(min < 0 || min >= 256) throw std::invalid_argument("Colormap min argument must be between 0 and 255");

    // Enable format stage
    cfg.enableFormat = true;

    // Set type format
    cfg.formatConfig.colormap = colormap;
    cfg.formatConfig.colormapMin = min;
    cfg.formatConfig.colormapMax = max;
    return *this;
}

ImageManipConfig& ImageManipConfig::setHorizontalFlip(bool flip) {
    // Enable format stage
    cfg.enableFormat = true;

    // Set pixel format
    cfg.formatConfig.flipHorizontal = flip;
    return *this;
}

void ImageManipConfig::setVerticalFlip(bool flip) {
    // Enable format stage
    cfg.enableFormat = true;

    // Set pixel format
    cfg.formatConfig.flipVertical = flip;
}

ImageManipConfig& ImageManipConfig::setReusePreviousImage(bool reuse) {
    cfg.reusePreviousImage = reuse;
    return *this;
}

ImageManipConfig& ImageManipConfig::setSkipCurrentImage(bool skip) {
    cfg.skipCurrentImage = skip;
    return *this;
}

ImageManipConfig& ImageManipConfig::setKeepAspectRatio(bool keep) {
    // Set whether to keep aspect ratio or not
    cfg.resizeConfig.keepAspectRatio = keep;
    return *this;
}

ImageManipConfig& ImageManipConfig::setInterpolation(dai::Interpolation interpolation) {
    cfg.interpolation = interpolation;
    return *this;
}

// Functions to retrieve properties
float ImageManipConfig::getCropXMin() const {
    return cfg.cropConfig.cropRect.xmin;
}

float ImageManipConfig::getCropYMin() const {
    return cfg.cropConfig.cropRect.ymin;
}

float ImageManipConfig::getCropXMax() const {
    return cfg.cropConfig.cropRect.xmax;
}

float ImageManipConfig::getCropYMax() const {
    return cfg.cropConfig.cropRect.ymax;
}

int ImageManipConfig::getResizeWidth() const {
    return cfg.resizeConfig.width;
}

int ImageManipConfig::getResizeHeight() const {
    return cfg.resizeConfig.height;
}

ImageManipConfig::CropConfig ImageManipConfig::getCropConfig() const {
    return cfg.cropConfig;
}

ImageManipConfig::ResizeConfig ImageManipConfig::getResizeConfig() const {
    return cfg.resizeConfig;
}

ImageManipConfig::FormatConfig ImageManipConfig::getFormatConfig() const {
    return cfg.formatConfig;
}

bool ImageManipConfig::isResizeThumbnail() const {
    return cfg.resizeConfig.lockAspectRatioFill;
}

dai::Colormap ImageManipConfig::getColormap() const {
    return cfg.formatConfig.colormap;
}

dai::RawImageManipConfig ImageManipConfig::get() const {
    return cfg;
}

ImageManipConfig& ImageManipConfig::set(dai::RawImageManipConfig config) {
    cfg = config;
    return *this;
}

dai::Interpolation ImageManipConfig::getInterpolation() const {
    return cfg.interpolation;
}

}  // namespace dai
