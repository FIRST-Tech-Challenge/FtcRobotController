#pragma once
#include <cstdint>
#include <vector>

#include "RawImgFrame.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"

// shared
#include "depthai-shared/common/Colormap.hpp"
#include "depthai-shared/common/Interpolation.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/RotatedRect.hpp"
#include "depthai-shared/common/Size2f.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawImageManipConfig structure
struct RawImageManipConfig : public RawBuffer {
    // NNData data is in PoBuf
    struct CropRect {
        // Normalized range 0-1
        float xmin = 0.0f, ymin = 0.0f, xmax = 0.0f, ymax = 0.0f;

        DEPTHAI_SERIALIZE(CropRect, xmin, ymin, xmax, ymax);
    };

    struct CropConfig {
        CropRect cropRect;
        RotatedRect cropRotatedRect;

        bool enableCenterCropRectangle = false;
        // if enableCenterCropRectangle -> automatically calculated crop parameters
        float cropRatio = 1.0f, widthHeightAspectRatio = 1.0f;

        bool enableRotatedRect = false;

        // Range 0..1 by default. Set 'false' to specify in pixels
        bool normalizedCoords = true;

        DEPTHAI_SERIALIZE(
            CropConfig, cropRect, cropRotatedRect, enableCenterCropRectangle, cropRatio, widthHeightAspectRatio, enableRotatedRect, normalizedCoords);
    };

    struct ResizeConfig {
        int width = 0, height = 0;
        bool lockAspectRatioFill = false;
        char bgRed = 0, bgGreen = 0, bgBlue = 0;

        //  clockwise order, pt[0] is mapped to the top-left output corner
        std::vector<Point2f> warpFourPoints;
        bool normalizedCoords = true;
        bool enableWarp4pt = false;

        std::vector<float> warpMatrix3x3;
        bool enableWarpMatrix = false;

        // Warp background / border mode: replicates pixels if true,
        // otherwise fills with a constant color defined by: bgRed, bgGreen, bgBlue
        bool warpBorderReplicate = false;

        // clockwise
        float rotationAngleDeg;
        bool enableRotation = false;

        /**
         * Whether to keep aspect ratio of input or not
         */
        bool keepAspectRatio = true;

        DEPTHAI_SERIALIZE(ResizeConfig,
                          width,
                          height,
                          lockAspectRatioFill,
                          bgRed,
                          bgGreen,
                          bgBlue,
                          warpFourPoints,
                          normalizedCoords,
                          enableWarp4pt,
                          warpMatrix3x3,
                          enableWarpMatrix,
                          warpBorderReplicate,
                          rotationAngleDeg,
                          enableRotation,
                          keepAspectRatio);
    };

    struct FormatConfig {
        RawImgFrame::Type type = RawImgFrame::Type::NONE;
        bool flipHorizontal = false;
        bool flipVertical = false;
        Colormap colormap = Colormap::NONE;
        int colormapMin = 0;
        int colormapMax = 255;

        DEPTHAI_SERIALIZE(FormatConfig, type, flipHorizontal, flipVertical, colormap, colormapMin, colormapMax);
    };

    CropConfig cropConfig;
    ResizeConfig resizeConfig;
    FormatConfig formatConfig;

    bool enableCrop = false;
    bool enableResize = false;
    bool enableFormat = false;

    // Usable with runtime config only,
    // when ImageManipProperties.inputConfig.setWaitForMessage(true) is set
    bool reusePreviousImage = false;
    bool skipCurrentImage = false;

    /// Interpolation type to use
    Interpolation interpolation = Interpolation::AUTO;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageManipConfig;
    };

    DEPTHAI_SERIALIZE(RawImageManipConfig,
                      cropConfig,
                      resizeConfig,
                      formatConfig,
                      enableCrop,
                      enableResize,
                      enableFormat,
                      reusePreviousImage,
                      skipCurrentImage,
                      interpolation);
};

}  // namespace dai
