#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawCameraControl structure
struct RawCameraControl : public RawBuffer {
    enum class Command : uint8_t {
        START_STREAM = 1,
        STOP_STREAM = 2,
        STILL_CAPTURE = 3,
        MOVE_LENS = 4, /* [1] lens position: 0-255
                        */
        AF_TRIGGER = 5,
        AE_MANUAL = 6, /* [1] exposure time [us]
                        * [2] sensitivity [iso]
                        * [3] frame duration [us]
                        */
        AE_AUTO = 7,
        AWB_MODE = 8,                  /* [1] awb_mode: AutoWhiteBalanceMode
                                        */
        SCENE_MODE = 9,                /* [1] scene_mode: SceneMode
                                        */
        ANTIBANDING_MODE = 10,         /* [1] antibanding_mode: AntiBandingMode
                                        */
        EXPOSURE_COMPENSATION = 11,    /* [1] value
                                        */
        AE_LOCK = 13,                  /* [1] ae_lock_mode: bool
                                        */
        AE_TARGET_FPS_RANGE = 14,      /* [1] min_fps
                                        * [2] max_fps
                                        */
        AWB_LOCK = 16,                 /* [1] awb_lock_mode: bool
                                        */
        CAPTURE_INTENT = 17,           /* [1] capture_intent_mode: CaptureIntent
                                        */
        CONTROL_MODE = 18,             /* [1] control_mode: ControlMode
                                        */
        FRAME_DURATION = 21,           /* [1] frame_duration
                                        */
        SENSITIVITY = 23,              /* [1] iso_val
                                        */
        EFFECT_MODE = 24,              /* [1] effect_mode: EffectMode
                                        */
        AF_MODE = 26,                  /* [1] af_mode: AutoFocusMode
                                        */
        NOISE_REDUCTION_STRENGTH = 27, /* [1] value
                                        */
        SATURATION = 28,               /* [1] value
                                        */
        BRIGHTNESS = 31,               /* [1] value
                                        */
        STREAM_FORMAT = 33,            /* [1] format
                                        */
        RESOLUTION = 34,               /* [1] width
                                        * [2] height
                                        */
        SHARPNESS = 35,                /* [1] value
                                        */
        CUSTOM_USECASE = 40,           /* [1] value
                                        */
        CUSTOM_CAPT_MODE = 41,         /* [1] value
                                        */
        CUSTOM_EXP_BRACKETS = 42,      /* [1] val1
                                        * [2] val2
                                        * [3] val3
                                        */
        CUSTOM_CAPTURE = 43,           /* [1] value
                                        */
        CONTRAST = 44,                 /* [1] value
                                        */
        AE_REGION = 45,                /* [1] x
                                        * [2] y
                                        * [3] width
                                        * [4] height
                                        * [5] priority
                                        */
        AF_REGION = 46,                /* [1] x
                                        * [2] y
                                        * [3] width
                                        * [4] height
                                        * [5] priority
                                        */
        LUMA_DENOISE = 47,             /* [1] value
                                        */
        CHROMA_DENOISE = 48,           /* [1] value
                                        */
        WB_COLOR_TEMP = 49,            /* [1] value
                                        */
        EXTERNAL_TRIGGER = 50,
        AF_LENS_RANGE = 51,
        FRAME_SYNC = 52,
        STROBE_CONFIG = 53,
        STROBE_TIMINGS = 54,
    };

    enum class AutoFocusMode : uint8_t {
        /**
         * Autofocus disabled. Suitable for manual focus
         */
        OFF = 0,
        /**
         * Basic automatic focus mode. In this mode, the lens does not move unless the autofocus trigger action is called.
         */
        AUTO,
        /**
         * Close-up focusing mode - this mode is optimized for focusing on objects very close to the camera.
         */
        MACRO,
        /**
         * In this mode, the AF algorithm modifies the lens position continually to attempt to provide a constantly-in-focus image stream.
         * The focusing behavior should be suitable for good quality video recording; typically this means slower focus movement and no overshoots.
         */
        CONTINUOUS_VIDEO,
        /**
         * In this mode, the AF algorithm modifies the lens position continually to attempt to provide a constantly-in-focus image stream.
         * The focusing behavior should be suitable for still image capture; typically this means focusing as fast as possible
         */
        CONTINUOUS_PICTURE,
        /**
         * Extended depth of field (digital focus) mode. The camera device will produce images with an extended depth of field automatically.
         * AF triggers are ignored.
         */
        EDOF,
    };

    enum class AutoWhiteBalanceMode : uint8_t {
        /**
         * The camera device's auto-white balance routine is disabled.
         */
        OFF = 0,
        /**
         * The camera device's auto-white balance routine is active.
         */
        AUTO,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses incandescent light as the assumed scene illumination for white
         * balance.
         */
        INCANDESCENT,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses fluorescent light as the assumed scene illumination for white
         * balance.
         */
        FLUORESCENT,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses warm fluorescent light as the assumed scene illumination for white
         * balance.
         */
        WARM_FLUORESCENT,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses daylight light as the assumed scene illumination for white
         * balance.
         */
        DAYLIGHT,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses cloudy daylight light as the assumed scene illumination for white
         * balance.
         */
        CLOUDY_DAYLIGHT,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses twilight light as the assumed scene illumination for white
         * balance.
         */
        TWILIGHT,
        /**
         * The camera device's auto-white balance routine is disabled; the camera device uses shade light as the assumed scene illumination for white balance.
         */
        SHADE,
    };

    enum class SceneMode : uint8_t {
        /**
         * Indicates that no scene modes are set for a given capture request.
         */
        UNSUPPORTED = 0,
        /**
         * If face detection support exists, use face detection data for auto-focus, auto-white balance, and auto-exposure routines.
         */
        FACE_PRIORITY,
        /**
         * Optimized for photos of quickly moving objects. Similar to SPORTS scene mode.
         */
        ACTION,
        /**
         * Optimized for still photos of people.
         */
        PORTRAIT,
        /**
         * Optimized for photos of distant macroscopic objects.
         */
        LANDSCAPE,
        /**
         * Optimized for low-light settings.
         */
        NIGHT,
        /**
         * Optimized for still photos of people in low-light settings.
         */
        NIGHT_PORTRAIT,
        /**
         * Optimized for dim, indoor settings where flash must remain off.
         */
        THEATRE,
        /**
         * Optimized for bright, outdoor beach settings.
         */
        BEACH,
        /**
         * Optimized for bright, outdoor settings containing snow.
         */
        SNOW,
        /**
         * Optimized for scenes of the setting sun.
         */
        SUNSET,
        /**
         * Optimized to avoid blurry photos due to small amounts of device motion (for example: due to hand shake).
         */
        STEADYPHOTO,
        /**
         * Optimized for nighttime photos of fireworks.
         */
        FIREWORKS,
        /**
         * Optimized for photos of quickly moving people.
         */
        SPORTS,
        /**
         * Optimized for dim, indoor settings with multiple moving people.
         */
        PARTY,
        /**
         * Optimized for dim settings where the main light source is a candle.
         */
        CANDLELIGHT,
        /**
         * Optimized for accurately capturing a photo of barcode for use by camera applications that wish to read the barcode value.
         */
        BARCODE,
    };

    enum class AntiBandingMode : uint8_t {
        /**
         * The camera device will not adjust exposure duration to avoid banding problems.
         */
        OFF = 0,
        /**
         * The camera device will adjust exposure duration to avoid banding problems with 50Hz illumination sources.
         */
        MAINS_50_HZ,
        /**
         * The camera device will adjust exposure duration to avoid banding problems with 60Hz illumination sources.
         */
        MAINS_60_HZ,
        /**
         * The camera device will automatically adapt its antibanding routine to the current illumination condition. This is the default mode if AUTO is
         * available on given camera device.
         */
        AUTO,
    };

    enum class CaptureIntent : uint8_t {
        /**
         * The goal of this request doesn't fall into the other categories. The camera device will default to preview-like behavior.
         */
        CUSTOM = 0,
        /**
         * This request is for a preview-like use case.
         */
        PREVIEW,
        /**
         * This request is for a still capture-type use case.
         */
        STILL_CAPTURE,
        /**
         * This request is for a video recording use case.
         */
        VIDEO_RECORD,
        /**
         * This request is for a video snapshot (still image while recording video) use case.
         * The camera device should take the highest-quality image possible (given the other settings)
         * without disrupting the frame rate of video recording.
         */
        VIDEO_SNAPSHOT,
        /**
         * This request is for a ZSL usecase; the application will stream full-resolution images and reprocess one or several later for a final capture.
         */
        ZERO_SHUTTER_LAG,
    };

    enum class ControlMode : uint8_t {
        /**
         * Full application control of pipeline. All control by the device's metering and focusing (3A) routines is disabled.
         */
        OFF = 0,
        /**
         * Use settings for each individual 3A routine. Manual control of capture parameters is disabled.
         */
        AUTO,
        /**
         * Use a specific scene mode. Enabling this disables Auto-Exposure, AWB and AF controls;
         */
        USE_SCENE_MODE,
    };

    enum class EffectMode : uint8_t {
        /**
         *  No color effect will be applied.
         */
        OFF = 0,
        /**
         * A "monocolor" effect where the image is mapped into a single color. This will typically be grayscale.
         */
        MONO,
        /**
         * A "photo-negative" effect where the image's colors are inverted.
         */
        NEGATIVE,
        /**
         * A "solarisation" effect (Sabattier effect) where the image is wholly or partially reversed in tone.
         */
        SOLARIZE,
        /**
         * A "sepia" effect where the image is mapped into warm gray, red, and brown tones.
         */
        SEPIA,
        /**
         * A "posterization" effect where the image uses discrete regions of tone rather than a continuous gradient of tones.
         */
        POSTERIZE,
        /**
         * A "whiteboard" effect where the image is typically displayed as regions of white, with black or grey details.
         */
        WHITEBOARD,
        /**
         * A "blackboard" effect where the image is typically displayed as regions of black, with white or grey details.
         */
        BLACKBOARD,
        /**
         * An "aqua" effect where a blue hue is added to the image.
         */
        AQUA,
    };

    enum class FrameSyncMode : uint8_t {
        OFF = 0,
        OUTPUT,
        INPUT,
        // TODO soft sync modes?
    };

    struct ManualExposureParams {
        uint32_t exposureTimeUs;
        uint32_t sensitivityIso;
        uint32_t frameDurationUs;

        DEPTHAI_SERIALIZE(ManualExposureParams, exposureTimeUs, sensitivityIso, frameDurationUs);
    };

    // AE_REGION / AF_REGION
    struct RegionParams {
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        // Set to 1 for now. TODO
        uint32_t priority;

        DEPTHAI_SERIALIZE(RegionParams, x, y, width, height, priority);
    };

    struct StrobeTimings {
        /// Start offset in microseconds, relative to exposure window
        int32_t exposureBeginOffsetUs;
        /// End offset in microseconds, relative to exposure window
        int32_t exposureEndOffsetUs;
        /// Fixed duration in microseconds. If set (non-zero), overrides `exposureEndOffsetUs`
        uint32_t durationUs;

        DEPTHAI_SERIALIZE(StrobeTimings, exposureBeginOffsetUs, exposureEndOffsetUs, durationUs);
    };

    struct StrobeConfig {
        /// Enable strobe output
        uint8_t enable;
        /// 1 for normal polarity (high-active), 0 otherwise
        uint8_t activeLevel;
        /// GPIO number to drive, or -1 if sensor driven
        int8_t gpioNumber;

        DEPTHAI_SERIALIZE(StrobeConfig, enable, activeLevel, gpioNumber);
    };

    uint64_t cmdMask = 0;

    AutoFocusMode autoFocusMode = AutoFocusMode::CONTINUOUS_VIDEO;

    /**
     * Lens/VCM position, range: 0..255. Used with `autoFocusMode = OFF`.
     * With current IMX378 modules:
     * - max 255: macro focus, at 8cm distance
     * - infinite focus at about 120..130 (may vary from module to module)
     * - lower values lead to out-of-focus (lens too close to the sensor array)
     */
    uint8_t lensPosition = 0;
    uint8_t lensPosAutoInfinity, lensPosAutoMacro;

    ManualExposureParams expManual;
    RegionParams aeRegion, afRegion;
    AutoWhiteBalanceMode awbMode;
    SceneMode sceneMode;
    AntiBandingMode antiBandingMode;
    EffectMode effectMode;
    FrameSyncMode frameSyncMode;
    StrobeConfig strobeConfig;
    StrobeTimings strobeTimings;
    bool aeLockMode;
    bool awbLockMode;
    int8_t expCompensation;  //  -9 ..  9
    int8_t brightness;       // -10 .. 10
    int8_t contrast;         // -10 .. 10
    int8_t saturation;       // -10 .. 10
    uint8_t sharpness;       //   0 ..  4
    uint8_t lumaDenoise;     //   0 ..  4
    uint8_t chromaDenoise;   //   0 ..  4
    uint16_t wbColorTemp;    // 1000 .. 12000
    uint8_t lowPowerNumFramesBurst;
    uint8_t lowPowerNumFramesDiscard;

    void setCommand(Command cmd, bool value = true) {
        uint64_t mask = 1ull << (uint8_t)cmd;
        if(value) {
            cmdMask |= mask;
        } else {
            cmdMask &= ~mask;
        }
    }
    void clearCommand(Command cmd) {
        setCommand(cmd, false);
    }
    bool getCommand(Command cmd) {
        return !!(cmdMask & (1ull << (uint8_t)cmd));
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::CameraControl;
    };

    DEPTHAI_SERIALIZE(RawCameraControl,
                      cmdMask,
                      autoFocusMode,
                      lensPosition,
                      lensPosAutoInfinity,
                      lensPosAutoMacro,
                      expManual,
                      aeRegion,
                      afRegion,
                      awbMode,
                      sceneMode,
                      antiBandingMode,
                      aeLockMode,
                      awbLockMode,
                      effectMode,
                      frameSyncMode,
                      strobeConfig,
                      strobeTimings,
                      expCompensation,
                      brightness,
                      contrast,
                      saturation,
                      sharpness,
                      lumaDenoise,
                      chromaDenoise,
                      wbColorTemp,
                      lowPowerNumFramesBurst,
                      lowPowerNumFramesDiscard);
};

}  // namespace dai
