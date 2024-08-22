#pragma once

#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraImageOrientation.hpp"
#include "depthai-shared/common/FrameEvent.hpp"
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 *  Specify properties for ColorCamera such as camera ID, ...
 */
struct ColorCameraProperties : PropertiesSerializable<Properties, ColorCameraProperties> {
    static constexpr int AUTO = -1;

    struct IspScale {
        int32_t horizNumerator = 0;
        int32_t horizDenominator = 0;
        int32_t vertNumerator = 0;
        int32_t vertDenominator = 0;

        DEPTHAI_SERIALIZE(IspScale, horizNumerator, horizDenominator, vertNumerator, vertDenominator);
    };

    /**
     * Select the camera sensor resolution
     */
    enum class SensorResolution : int32_t {
        /// 1920 × 1080
        THE_1080_P,
        /// 3840 × 2160
        THE_4_K,
        /// 4056 × 3040
        THE_12_MP,
        /// 4208 × 3120
        THE_13_MP,
        /// 1280 × 720
        THE_720_P,
        /// 1280 × 800
        THE_800_P,
        /// 1920 × 1200
        THE_1200_P,
        /// 2592 × 1944
        THE_5_MP,
        /// 4000 × 3000
        THE_4000X3000,
        /// 5312 × 6000
        THE_5312X6000,
        /// 8000 × 6000
        THE_48_MP,
        /// 1440 × 1080
        THE_1440X1080,
        /// 1352 × 1012
        THE_1352X1012,
        /// 2024 × 1520
        THE_2024X1520,
    };

    /**
     * For 24 bit color these can be either RGB or BGR
     */
    enum class ColorOrder : int32_t { BGR, RGB };

    /*
     * Initial controls applied to ColorCamera node
     */
    RawCameraControl initialControl;

    /**
     * Which socket will color camera use
     */
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;

    /**
     * Which camera name will color camera use
     */
    std::string cameraName = "";

    /**
     * Camera sensor image orientation / pixel readout
     */
    CameraImageOrientation imageOrientation = CameraImageOrientation::AUTO;

    /**
     * For 24 bit color these can be either RGB or BGR
     */
    ColorOrder colorOrder = ColorOrder::BGR;
    /**
     * Are colors interleaved (R1G1B1, R2G2B2, ...) or planar (R1R2..., G1G2..., B1B2)
     */
    bool interleaved = true;
    /**
     * Are values FP16 type (0.0 - 255.0)
     */
    bool fp16 = false;

    /**
     * Preview frame output height
     */
    uint32_t previewHeight = 300;
    /**
     * Preview frame output width
     */
    uint32_t previewWidth = 300;

    /**
     * Preview frame output width
     */
    int32_t videoWidth = AUTO;

    /**
     * Preview frame output height
     */
    int32_t videoHeight = AUTO;

    /**
     * Preview frame output width
     */
    int32_t stillWidth = AUTO;

    /**
     * Preview frame output height
     */
    int32_t stillHeight = AUTO;

    /**
     * Select the camera sensor resolution
     */
    SensorResolution resolution = SensorResolution::THE_1080_P;
    /**
     * Camera sensor FPS
     */
    float fps = 30.0;

    /**
     * Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls etc.).
     * Default (0) matches the camera FPS, meaning that 3A is running on each frame.
     * Reducing the rate of 3A reduces the CPU usage on CSS, but also increases the convergence rate of 3A.
     * Note that camera controls will be processed at this rate. E.g. if camera is running at 30 fps, and camera control is sent at every frame,
     * but 3A fps is set to 15, the camera control messages will be processed at 15 fps rate, which will lead to queueing.

     */
    int isp3aFps = 0;

    /**
     * Initial sensor crop, -1 signifies center crop
     */
    float sensorCropX = AUTO;
    float sensorCropY = AUTO;

    /**
     * Whether to keep aspect ratio of input (video size) or not
     */
    bool previewKeepAspectRatio = true;

    /**
     * Configure scaling for `isp` output.
     */
    IspScale ispScale;

    /**
     * Pool sizes
     */
    int numFramesPoolRaw = 3;
    int numFramesPoolIsp = 3;
    int numFramesPoolVideo = 4;
    int numFramesPoolPreview = 4;
    int numFramesPoolStill = 4;

    /**
     * List of events to receive, the rest will be ignored
     */
    std::vector<dai::FrameEvent> eventFilter = {dai::FrameEvent::READOUT_START};

    /**
     * Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
     * The packed format is more efficient, consuming less memory on device, and less data
     * to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on 3 bytes.
     * When packing is disabled (`false`), data is saved lsb-aligned, e.g. a RAW10 pixel
     * will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
     * Default is auto: enabled for standard color/monochrome cameras where ISP can work
     * with both packed/unpacked, but disabled for other cameras like ToF.
     */
    tl::optional<bool> rawPacked;
};

DEPTHAI_SERIALIZE_EXT(ColorCameraProperties,
                      initialControl,
                      boardSocket,
                      cameraName,
                      imageOrientation,
                      colorOrder,
                      interleaved,
                      fp16,
                      previewHeight,
                      previewWidth,
                      videoWidth,
                      videoHeight,
                      stillWidth,
                      stillHeight,
                      resolution,
                      fps,
                      isp3aFps,
                      sensorCropX,
                      sensorCropY,
                      previewKeepAspectRatio,
                      ispScale,
                      numFramesPoolRaw,
                      numFramesPoolIsp,
                      numFramesPoolVideo,
                      numFramesPoolPreview,
                      numFramesPoolStill,
                      rawPacked);

}  // namespace dai
