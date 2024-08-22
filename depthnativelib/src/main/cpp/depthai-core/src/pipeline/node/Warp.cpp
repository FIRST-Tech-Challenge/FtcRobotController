#include "depthai/pipeline/node/Warp.hpp"
namespace dai {
namespace node {

Warp::Warp(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Warp(par, nodeId, std::make_unique<Warp::Properties>()) {}
Warp::Warp(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Warp, WarpProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&inputImage});
    setOutputRefs({&out});
}

Warp::Properties& Warp::getProperties() {
    // properties.initialConfig = *rawConfig;
    return properties;
}

void Warp::setOutputSize(std::tuple<int, int> size) {
    properties.outputWidth = std::get<0>(size);
    properties.outputHeight = std::get<1>(size);
}

void Warp::setOutputSize(int width, int height) {
    setOutputSize({width, height});
}

void Warp::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void Warp::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

void Warp::setWarpMesh(const float* meshData, int numMeshPoints, int width, int height) {
    if(numMeshPoints < width * height) {
        throw std::invalid_argument("Not enough points provided for specified width and height");
    }

    // TODO(themarpe) - optimize
    Asset asset("mesh");
    asset.alignment = 64;

    // Align stride to 16B
    constexpr auto ALIGNMENT = 16;
    size_t meshStride = ((size_t)((sizeof(Point2f) * width)) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1);
    // Specify final mesh size
    size_t meshSize = meshStride * height;

    // Create mesh data
    asset.data = std::vector<uint8_t>(meshSize);

    // Fill out mesh points with stride
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            // get location in meshData
            size_t inputMeshIndex = (i * width + j) * 2;  // 2 float values per point

            // get output offset
            size_t outputMeshOffset = (meshStride * i) + (j * sizeof(Point2f));
            auto& point = reinterpret_cast<Point2f&>(asset.data.data()[outputMeshOffset]);

            // Asign reversed mesh coordinates (HW specified)
            point.x = meshData[inputMeshIndex + 1];
            point.y = meshData[inputMeshIndex + 0];
        }
    }

    properties.meshUri = assetManager.set(asset)->getRelativeUri();
    properties.meshWidth = width;
    properties.meshHeight = height;
}

void Warp::setWarpMesh(const std::vector<Point2f>& meshData, int width, int height) {
    setWarpMesh(reinterpret_cast<const float*>(meshData.data()), static_cast<int>(meshData.size()), width, height);
}

void Warp::setWarpMesh(const std::vector<std::pair<float, float>>& meshData, int width, int height) {
    setWarpMesh(reinterpret_cast<const float*>(meshData.data()), static_cast<int>(meshData.size()), width, height);
}

void Warp::setHwIds(std::vector<int> ids) {
    properties.warpHwIds = ids;
}

std::vector<int> Warp::getHwIds() const {
    return properties.warpHwIds;
}

void Warp::setInterpolation(dai::Interpolation interpolation) {
    properties.interpolation = interpolation;
}

dai::Interpolation Warp::getInterpolation() const {
    return properties.interpolation;
}

}  // namespace node
}  // namespace dai
