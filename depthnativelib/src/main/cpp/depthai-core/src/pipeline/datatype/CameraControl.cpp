#include "depthai/pipeline/datatype/CameraControl.hpp"

namespace dai {

std::shared_ptr<RawBuffer> CameraControl::serialize() const {
    return raw;
}

CameraControl::CameraControl() : Buffer(std::make_shared<RawCameraControl>()), cfg(*dynamic_cast<RawCameraControl*>(raw.get())) {}
CameraControl::CameraControl(std::shared_ptr<RawCameraControl> ptr) : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawCameraControl*>(raw.get())) {}

// helpers
// Functions to set properties
CameraControl& CameraControl::setCaptureStill(bool capture) {
    // Enable capture
    cfg.setCommand(RawCameraControl::Command::STILL_CAPTURE, capture);
    return *this;
}

CameraControl& CameraControl::setStartStreaming() {
    cfg.setCommand(RawCameraControl::Command::START_STREAM);
    return *this;
}
CameraControl& CameraControl::setStopStreaming() {
    cfg.setCommand(RawCameraControl::Command::STOP_STREAM);
    return *this;
}
CameraControl& CameraControl::setExternalTrigger(int numFramesBurst, int numFramesDiscard) {
    cfg.setCommand(RawCameraControl::Command::EXTERNAL_TRIGGER);
    cfg.lowPowerNumFramesBurst = numFramesBurst;
    cfg.lowPowerNumFramesDiscard = numFramesDiscard;
    return *this;
}

CameraControl& CameraControl::setFrameSyncMode(FrameSyncMode mode) {
    cfg.setCommand(RawCameraControl::Command::FRAME_SYNC);
    cfg.frameSyncMode = mode;
    return *this;
}

CameraControl& CameraControl::setStrobeSensor(int activeLevel) {
    cfg.setCommand(RawCameraControl::Command::STROBE_CONFIG);
    cfg.strobeConfig.enable = true;
    cfg.strobeConfig.activeLevel = activeLevel;
    cfg.strobeConfig.gpioNumber = -1;
    return *this;
}

CameraControl& CameraControl::setStrobeExternal(int gpioNumber, int activeLevel) {
    cfg.setCommand(RawCameraControl::Command::STROBE_CONFIG);
    cfg.strobeConfig.enable = true;
    cfg.strobeConfig.activeLevel = activeLevel;
    cfg.strobeConfig.gpioNumber = gpioNumber;
    return *this;
}

CameraControl& CameraControl::setStrobeDisable() {
    cfg.setCommand(RawCameraControl::Command::STROBE_CONFIG);
    cfg.strobeConfig.enable = false;
    return *this;
}

// Focus
CameraControl& CameraControl::setAutoFocusMode(AutoFocusMode mode) {
    cfg.setCommand(RawCameraControl::Command::AF_MODE);
    cfg.autoFocusMode = mode;
    return *this;
}
CameraControl& CameraControl::setAutoFocusTrigger() {
    cfg.setCommand(RawCameraControl::Command::AF_TRIGGER);
    return *this;
}
CameraControl& CameraControl::setAutoFocusLensRange(int infinityPosition, int macroPosition) {
    cfg.setCommand(RawCameraControl::Command::AF_LENS_RANGE);
    cfg.lensPosAutoInfinity = infinityPosition;
    cfg.lensPosAutoMacro = macroPosition;
    return *this;
}
CameraControl& CameraControl::setAutoFocusRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height) {
    cfg.setCommand(RawCameraControl::Command::AF_REGION);
    cfg.afRegion.x = startX;
    cfg.afRegion.y = startY;
    cfg.afRegion.width = width;
    cfg.afRegion.height = height;
    cfg.afRegion.priority = 1;  // TODO
    return *this;
}
CameraControl& CameraControl::setManualFocus(uint8_t lensPosition) {
    cfg.setCommand(RawCameraControl::Command::MOVE_LENS);
    cfg.lensPosition = lensPosition;
    return *this;
}

// Exposure
CameraControl& CameraControl::setAutoExposureEnable() {
    cfg.setCommand(RawCameraControl::Command::AE_AUTO);
    return *this;
}
CameraControl& CameraControl::setAutoExposureLock(bool lock) {
    cfg.setCommand(RawCameraControl::Command::AE_LOCK);
    cfg.aeLockMode = lock;
    return *this;
}
CameraControl& CameraControl::setAutoExposureRegion(uint16_t startX, uint16_t startY, uint16_t width, uint16_t height) {
    cfg.setCommand(RawCameraControl::Command::AE_REGION);
    cfg.aeRegion.x = startX;
    cfg.aeRegion.y = startY;
    cfg.aeRegion.width = width;
    cfg.aeRegion.height = height;
    cfg.aeRegion.priority = 1;  // TODO
    return *this;
}
CameraControl& CameraControl::setAutoExposureCompensation(int compensation) {
    cfg.setCommand(RawCameraControl::Command::EXPOSURE_COMPENSATION);
    cfg.expCompensation = compensation;
    return *this;
}
CameraControl& CameraControl::setAntiBandingMode(AntiBandingMode mode) {
    cfg.setCommand(RawCameraControl::Command::ANTIBANDING_MODE);
    cfg.antiBandingMode = mode;
    return *this;
}
CameraControl& CameraControl::setManualExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso) {
    cfg.setCommand(RawCameraControl::Command::AE_MANUAL);
    cfg.expManual.exposureTimeUs = exposureTimeUs;
    cfg.expManual.sensitivityIso = sensitivityIso;
    cfg.expManual.frameDurationUs = 0;  // TODO
    return *this;
}

void CameraControl::setManualExposure(std::chrono::microseconds exposureTime, uint32_t sensitivityIso) {
    setManualExposure(exposureTime.count(), sensitivityIso);
}

// White Balance
CameraControl& CameraControl::setAutoWhiteBalanceMode(AutoWhiteBalanceMode mode) {
    cfg.setCommand(RawCameraControl::Command::AWB_MODE);
    cfg.awbMode = mode;
    return *this;
}
CameraControl& CameraControl::setAutoWhiteBalanceLock(bool lock) {
    cfg.setCommand(RawCameraControl::Command::AWB_LOCK);
    cfg.awbLockMode = lock;
    return *this;
}
CameraControl& CameraControl::setManualWhiteBalance(int colorTemperatureK) {
    cfg.setCommand(RawCameraControl::Command::WB_COLOR_TEMP);
    cfg.wbColorTemp = colorTemperatureK;
    return *this;
}

// Other image controls
CameraControl& CameraControl::setBrightness(int value) {
    cfg.setCommand(RawCameraControl::Command::BRIGHTNESS);
    cfg.brightness = value;
    return *this;
}
CameraControl& CameraControl::setContrast(int value) {
    cfg.setCommand(RawCameraControl::Command::CONTRAST);
    cfg.contrast = value;
    return *this;
}
CameraControl& CameraControl::setSaturation(int value) {
    cfg.setCommand(RawCameraControl::Command::SATURATION);
    cfg.saturation = value;
    return *this;
}
CameraControl& CameraControl::setSharpness(int value) {
    cfg.setCommand(RawCameraControl::Command::SHARPNESS);
    cfg.sharpness = value;
    return *this;
}
CameraControl& CameraControl::setLumaDenoise(int value) {
    cfg.setCommand(RawCameraControl::Command::LUMA_DENOISE);
    cfg.lumaDenoise = value;
    return *this;
}
CameraControl& CameraControl::setChromaDenoise(int value) {
    cfg.setCommand(RawCameraControl::Command::CHROMA_DENOISE);
    cfg.chromaDenoise = value;
    return *this;
}
CameraControl& CameraControl::setSceneMode(SceneMode mode) {
    cfg.setCommand(RawCameraControl::Command::SCENE_MODE);
    cfg.sceneMode = mode;
    return *this;
}
CameraControl& CameraControl::setEffectMode(EffectMode mode) {
    cfg.setCommand(RawCameraControl::Command::EFFECT_MODE);
    cfg.effectMode = mode;
    return *this;
}

bool CameraControl::getCaptureStill() const {
    return cfg.getCommand(RawCameraControl::Command::STILL_CAPTURE);
}

std::chrono::microseconds CameraControl::getExposureTime() const {
    return std::chrono::microseconds(cfg.expManual.exposureTimeUs);
}

int CameraControl::getSensitivity() const {
    return cfg.expManual.sensitivityIso;
}

int CameraControl::getLensPosition() const {
    return cfg.lensPosition;
}

dai::RawCameraControl CameraControl::get() const {
    return cfg;
}

CameraControl& CameraControl::set(dai::RawCameraControl config) {
    cfg = config;
    return *this;
}

}  // namespace dai
