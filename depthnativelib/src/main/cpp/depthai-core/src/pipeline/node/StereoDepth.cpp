#include "depthai/pipeline/node/StereoDepth.hpp"

// standard
#include <fstream>

#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

namespace dai {
namespace node {

StereoDepth::StereoDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : StereoDepth(par, nodeId, std::make_unique<StereoDepth::Properties>()) {}
StereoDepth::StereoDepth(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, StereoDepth, StereoDepthProperties>(par, nodeId, std::move(props)),
      rawConfig(std::make_shared<RawStereoDepthConfig>()),
      initialConfig(rawConfig) {
    // 'properties' defaults already set
    setInputRefs({&inputConfig, &left, &right});
    setOutputRefs({&depth,
                   &disparity,
                   &syncedLeft,
                   &syncedRight,
                   &rectifiedLeft,
                   &rectifiedRight,
                   &outConfig,
                   &debugDispLrCheckIt1,
                   &debugDispLrCheckIt2,
                   &debugExtDispLrCheckIt1,
                   &debugExtDispLrCheckIt2,
                   &debugDispCostDump,
                   &confidenceMap});

    setDefaultProfilePreset(presetMode);
}

StereoDepth::Properties& StereoDepth::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

void StereoDepth::setEmptyCalibration(void) {
    setRectification(false);
    logger::warn("{} is deprecated. This function call can be replaced by Stereo::setRectification(false). ", __func__);
}

void StereoDepth::loadMeshData(const std::vector<std::uint8_t>& dataLeft, const std::vector<std::uint8_t>& dataRight) {
    if(dataLeft.size() != dataRight.size()) {
        throw std::runtime_error("StereoDepth | left and right mesh sizes must match");
    }

    Asset meshAsset;
    std::string assetKey;
    meshAsset.alignment = 64;

    meshAsset.data = dataLeft;
    assetKey = "meshLeft";
    properties.mesh.meshLeftUri = assetManager.set(assetKey, meshAsset)->getRelativeUri();

    meshAsset.data = dataRight;
    assetKey = "meshRight";
    properties.mesh.meshRightUri = assetManager.set(assetKey, meshAsset)->getRelativeUri();

    properties.mesh.meshSize = static_cast<uint32_t>(meshAsset.data.size());
}

void StereoDepth::loadMeshFiles(const dai::Path& pathLeft, const dai::Path& pathRight) {
    std::ifstream streamLeft(pathLeft, std::ios::binary);
    if(!streamLeft.is_open()) {
        throw std::runtime_error(fmt::format("StereoDepth | Cannot open mesh at path: {}", pathLeft));
    }
    std::vector<std::uint8_t> dataLeft = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamLeft), {});

    std::ifstream streamRight(pathRight, std::ios::binary);
    if(!streamRight.is_open()) {
        throw std::runtime_error(fmt::format("StereoDepth | Cannot open mesh at path: {}", pathRight));
    }
    std::vector<std::uint8_t> dataRight = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamRight), {});

    loadMeshData(dataLeft, dataRight);
}

void StereoDepth::setMeshStep(int width, int height) {
    properties.mesh.stepWidth = width;
    properties.mesh.stepHeight = height;
}

void StereoDepth::setInputResolution(int width, int height) {
    properties.width = width;
    properties.height = height;
}
void StereoDepth::setInputResolution(std::tuple<int, int> resolution) {
    setInputResolution(std::get<0>(resolution), std::get<1>(resolution));
}
void StereoDepth::setOutputSize(int width, int height) {
    properties.outWidth = width;
    properties.outHeight = height;
}
void StereoDepth::setOutputKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
}
void StereoDepth::setMedianFilter(dai::MedianFilter median) {
    initialConfig.setMedianFilter(median);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setDepthAlign(Properties::DepthAlign align) {
    initialConfig.setDepthAlign(align);
    // Unset 'depthAlignCamera', that would take precedence otherwise
    properties.depthAlignCamera = CameraBoardSocket::AUTO;
}
void StereoDepth::setDepthAlign(CameraBoardSocket camera) {
    properties.depthAlignCamera = camera;
}
void StereoDepth::setConfidenceThreshold(int confThr) {
    initialConfig.setConfidenceThreshold(confThr);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setRectification(bool enable) {
    properties.enableRectification = enable;
}
void StereoDepth::setLeftRightCheck(bool enable) {
    initialConfig.setLeftRightCheck(enable);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setSubpixel(bool enable) {
    initialConfig.setSubpixel(enable);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setSubpixelFractionalBits(int subpixelFractionalBits) {
    initialConfig.setSubpixelFractionalBits(subpixelFractionalBits);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setExtendedDisparity(bool enable) {
    initialConfig.setExtendedDisparity(enable);
    properties.initialConfig = *rawConfig;
}
void StereoDepth::setRectifyEdgeFillColor(int color) {
    properties.rectifyEdgeFillColor = color;
}
void StereoDepth::setRectifyMirrorFrame(bool enable) {
    (void)enable;
    logger::warn("{} is deprecated.", __func__);
}
void StereoDepth::setOutputRectified(bool enable) {
    (void)enable;
    logger::warn("{} is deprecated. The output is auto-enabled if used", __func__);
}
void StereoDepth::setOutputDepth(bool enable) {
    (void)enable;
    logger::warn("{} is deprecated. The output is auto-enabled if used", __func__);
}

void StereoDepth::setRuntimeModeSwitch(bool enable) {
    properties.enableRuntimeStereoModeSwitch = enable;
}

void StereoDepth::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

float StereoDepth::getMaxDisparity() const {
    return initialConfig.getMaxDisparity();
}

void StereoDepth::setPostProcessingHardwareResources(int numShaves, int numMemorySlices) {
    properties.numPostProcessingShaves = numShaves;
    properties.numPostProcessingMemorySlices = numMemorySlices;
}

void StereoDepth::setFocalLengthFromCalibration(bool focalLengthFromCalibration) {
    properties.focalLengthFromCalibration = focalLengthFromCalibration;
}

void StereoDepth::useHomographyRectification(bool useHomographyRectification) {
    properties.useHomographyRectification = useHomographyRectification;
}

void StereoDepth::enableDistortionCorrection(bool enableDistortionCorrection) {
    useHomographyRectification(!enableDistortionCorrection);
}

void StereoDepth::setBaseline(float baseline) {
    properties.baseline = baseline;
}

void StereoDepth::setFocalLength(float focalLength) {
    properties.focalLength = focalLength;
}

void StereoDepth::setDisparityToDepthUseSpecTranslation(bool specTranslation) {
    properties.disparityToDepthUseSpecTranslation = specTranslation;
}

void StereoDepth::setRectificationUseSpecTranslation(bool specTranslation) {
    properties.rectificationUseSpecTranslation = specTranslation;
}

void StereoDepth::setDepthAlignmentUseSpecTranslation(bool specTranslation) {
    properties.depthAlignmentUseSpecTranslation = specTranslation;
}

void StereoDepth::setAlphaScaling(float alpha) {
    properties.alphaScaling = alpha;
}

void StereoDepth::setDefaultProfilePreset(PresetMode mode) {
    presetMode = mode;
    switch(presetMode) {
        case PresetMode::HIGH_ACCURACY: {
            initialConfig.setConfidenceThreshold(200);
            initialConfig.setLeftRightCheck(true);
            initialConfig.setLeftRightCheckThreshold(5);
        } break;
        case PresetMode::HIGH_DENSITY: {
            initialConfig.setConfidenceThreshold(245);
            initialConfig.setLeftRightCheck(true);
            initialConfig.setLeftRightCheckThreshold(10);
        } break;
    }
}

}  // namespace node
}  // namespace dai
