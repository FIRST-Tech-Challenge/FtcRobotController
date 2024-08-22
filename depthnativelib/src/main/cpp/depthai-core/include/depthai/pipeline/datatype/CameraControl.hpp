#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawCameraControl.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * CameraControl message.
 * Specifies various camera control commands like:
 *
 *  - Still capture
 *
 *  - Auto/manual focus
 *
 *  - Auto/manual white balance
 *
 *  - Auto/manual exposure
 *
 *  - Anti banding
 *
 *  - ...
 *
 *  By default the camera enables 3A, with auto-focus in `CONTINUOUS_VIDEO` mode,
 *  auto-white-balance in `AUTO` mode, and auto-exposure with anti-banding for
 *  50Hz mains frequency.
 *
 */
class CameraControl : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawCameraControl& cfg;

   public:
    using AutoFocusMode = RawCameraControl::AutoFocusMode;
    using AntiBandingMode = RawCameraControl::AntiBandingMode;
    using AutoWhiteBalanceMode = RawCameraControl::AutoWhiteBalanceMode;
    using SceneMode = RawCameraControl::SceneMode;
    using EffectMode = RawCameraControl::EffectMode;
    using FrameSyncMode = RawCameraControl::FrameSyncMode;

    /// Construct CameraControl message
    CameraControl();
    explicit CameraControl(std::shared_ptr<RawCameraControl> ptr);
    virtual ~CameraControl() = default;

    /**
     * Set a command to capture a still image
     */
    CameraControl& setCaptureStill(bool capture);

    /**
     * Set a command to start streaming
     */
    CameraControl& setStartStreaming();

    /**
     * Set a command to stop streaming
     */
    CameraControl& setStopStreaming();

    /**
     * Set a command to enable external trigger snapshot mode
     *
     * A rising edge on the sensor FSIN pin will make it capture a sequence of
     * `numFramesBurst` frames. First `numFramesDiscard` will be skipped as
     * configured (can be set to 0 as well), as they may have degraded quality
     */
    CameraControl& setExternalTrigger(int numFramesBurst, int numFramesDiscard);

    /**
     * Set the frame sync mode for continuous streaming operation mode,
     * translating to how the camera pin FSIN/FSYNC is used: input/output/disabled
     */
    CameraControl& setFrameSyncMode(FrameSyncMode mode);

    /**
     * Enable STROBE output on sensor pin, optionally configuring the polarity.
     * Note: for many sensors the polarity is high-active and not configurable
     */
    CameraControl& setStrobeSensor(int activeLevel = 1);

    /**
     * Enable STROBE output driven by a MyriadX GPIO, optionally configuring the polarity
     * This normally requires a FSIN/FSYNC/trigger input for MyriadX (usually GPIO 41),
     * to generate timings
     */
    CameraControl& setStrobeExternal(int gpioNumber, int activeLevel = 1);

    // TODO API to set strobe line directly high/low (not following the exposure window)
    // TODO API to set strobe timings, as offsets in relation to exposure window, or fixed duration

    /**
     * Disable STROBE output
     */
    CameraControl& setStrobeDisable();

    // Focus
    /**
     * Set a command to specify autofocus mode. Default `CONTINUOUS_VIDEO`
     */
    CameraControl& setAutoFocusMode(AutoFocusMode mode);

    /**
     * Set a command to trigger autofocus
     */
    CameraControl& setAutoFocusTrigger();

    /**
     * Set autofocus lens range, `infinityPosition < macroPosition`, valid values `0..255`.
     * May help to improve autofocus in case the lens adjustment is not typical/tuned
     */
    CameraControl& setAutoFocusLensRange(int infinityPosition, int macroPosition);

    /**
     * Set a command to specify focus region in pixels.
     * Note: the region should be mapped to the configured sensor resolution, before ISP scaling
     * @param startX X coordinate of top left corner of region
     * @param startY Y coordinate of top left corner of region
     * @param width Region width
     * @param height Region height
     */
    CameraControl& setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);

    /**
     * Set a command to specify manual focus position
     * @param lensPosition specify lens position 0..255
     */
    CameraControl& setManualFocus(uint8_t lensPosition);

    // Exposure
    /**
     * Set a command to enable auto exposure
     */
    CameraControl& setAutoExposureEnable();

    /**
     * Set a command to specify lock auto exposure
     * @param lock Auto exposure lock mode enabled or disabled
     */
    CameraControl& setAutoExposureLock(bool lock);

    /**
     * Set a command to specify auto exposure region in pixels.
     * Note: the region should be mapped to the configured sensor resolution, before ISP scaling
     * @param startX X coordinate of top left corner of region
     * @param startY Y coordinate of top left corner of region
     * @param width Region width
     * @param height Region height
     */
    CameraControl& setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height);

    /**
     * Set a command to specify auto exposure compensation
     * @param compensation Compensation value between -9..9, default 0
     */
    CameraControl& setAutoExposureCompensation(int compensation);

    /**
     * Set a command to specify anti-banding mode. Anti-banding / anti-flicker
     * works in auto-exposure mode, by controlling the exposure time to be applied
     * in multiples of half the mains period, for example in multiple of 10ms
     * for 50Hz (period 20ms) AC-powered illumination sources.
     *
     * If the scene would be too bright for the smallest exposure step
     * (10ms in the example, with ISO at a minimum of 100), anti-banding is not effective.
     *
     * @param mode Anti-banding mode to use. Default: `MAINS_50_HZ`
     */
    CameraControl& setAntiBandingMode(AntiBandingMode mode);

    /**
     * Set a command to manually specify exposure
     * @param exposureTimeUs Exposure time in microseconds
     * @param sensitivityIso Sensitivity as ISO value, usual range 100..1600
     */
    CameraControl& setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso);

    /**
     * Set a command to manually specify exposure
     * @param exposureTime Exposure time
     * @param sensitivityIso Sensitivity as ISO value, usual range 100..1600
     */
    void setManualExposure(std::chrono::microseconds exposureTime, uint32_t sensitivityIso);

    // White Balance
    /**
     * Set a command to specify auto white balance mode
     * @param mode Auto white balance mode to use. Default `AUTO`
     */
    CameraControl& setAutoWhiteBalanceMode(AutoWhiteBalanceMode mode);

    /**
     * Set a command to specify auto white balance lock
     * @param lock Auto white balance lock mode enabled or disabled
     */
    CameraControl& setAutoWhiteBalanceLock(bool lock);

    /**
     * Set a command to manually specify white-balance color correction
     * @param colorTemperatureK Light source color temperature in kelvins, range 1000..12000
     */
    CameraControl& setManualWhiteBalance(int colorTemperatureK);

    // Other image controls
    /**
     * Set a command to adjust image brightness
     * @param value Brightness, range -10..10, default 0
     */
    CameraControl& setBrightness(int value);

    /**
     * Set a command to adjust image contrast
     * @param value Contrast, range -10..10, default 0
     */
    CameraControl& setContrast(int value);

    /**
     * Set a command to adjust image saturation
     * @param value Saturation, range -10..10, default 0
     */
    CameraControl& setSaturation(int value);

    /**
     * Set a command to adjust image sharpness
     * @param value Sharpness, range 0..4, default 1
     */
    CameraControl& setSharpness(int value);

    /**
     * Set a command to adjust luma denoise amount
     * @param value Luma denoise amount, range 0..4, default 1
     */
    CameraControl& setLumaDenoise(int value);

    /**
     * Set a command to adjust chroma denoise amount
     * @param value Chroma denoise amount, range 0..4, default 1
     */
    CameraControl& setChromaDenoise(int value);

    /**
     * Set a command to specify scene mode
     * @param mode Scene mode
     */
    CameraControl& setSceneMode(SceneMode mode);

    /**
     * Set a command to specify effect mode
     * @param mode Effect mode
     */
    CameraControl& setEffectMode(EffectMode mode);

    // Functions to retrieve properties
    /**
     * Check whether command to capture a still is set
     * @returns True if capture still command is set
     */
    bool getCaptureStill() const;

    /**
     * Retrieves exposure time
     */
    std::chrono::microseconds getExposureTime() const;

    /**
     * Retrieves sensitivity, as an ISO value
     */
    int getSensitivity() const;

    /**
     * Retrieves lens position, range 0..255. Returns -1 if not available
     */
    int getLensPosition() const;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    CameraControl& set(dai::RawCameraControl config);

    /**
     * Retrieve configuration data for CameraControl.
     * @returns config for CameraControl
     */
    dai::RawCameraControl get() const;
};

}  // namespace dai
