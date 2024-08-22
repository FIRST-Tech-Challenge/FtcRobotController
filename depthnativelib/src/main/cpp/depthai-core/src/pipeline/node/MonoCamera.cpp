#include "depthai/pipeline/node/MonoCamera.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

MonoCamera::MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : MonoCamera(par, nodeId, std::make_unique<MonoCamera::Properties>()) {}
MonoCamera::MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, MonoCamera, MonoCameraProperties>(par, nodeId, std::move(props)),
      rawControl(std::make_shared<RawCameraControl>()),
      initialControl(rawControl) {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.resolution = MonoCameraProperties::SensorResolution::THE_720_P;
    properties.fps = 30.0;

    setInputRefs({&inputControl});
    setOutputRefs({&out, &raw, &frameEvent});
}

MonoCamera::Properties& MonoCamera::getProperties() {
    properties.initialControl = *rawControl;
    return properties;
}

// Set board socket to use
void MonoCamera::setBoardSocket(dai::CameraBoardSocket boardSocket) {
    properties.boardSocket = boardSocket;
}

// Get current board socket
CameraBoardSocket MonoCamera::getBoardSocket() const {
    return properties.boardSocket;
}

void MonoCamera::setCamera(std::string name) {
    properties.cameraName = name;
}

std::string MonoCamera::getCamera() const {
    return properties.cameraName;
}

// Set which color camera to use
void MonoCamera::setCamId(int64_t camId) {
    // cast to board socket
    switch(camId) {
        case 0:
            properties.boardSocket = CameraBoardSocket::CAM_A;
            break;
        case 1:
            properties.boardSocket = CameraBoardSocket::CAM_B;
            break;
        case 2:
            properties.boardSocket = CameraBoardSocket::CAM_C;
            break;
        case 3:
            properties.boardSocket = CameraBoardSocket::CAM_D;
            break;
        default:
            throw std::invalid_argument(fmt::format("CamId value: {} is invalid.", camId));
            break;
    }
}

// Get which color camera to use
int64_t MonoCamera::getCamId() const {
    return (int64_t)properties.boardSocket;
}

// Set camera image orientation
void MonoCamera::setImageOrientation(CameraImageOrientation imageOrientation) {
    properties.imageOrientation = imageOrientation;
}

// Get camera image orientation
CameraImageOrientation MonoCamera::getImageOrientation() const {
    // TODO: in case of AUTO, see if possible to return actual value determined by device?
    return properties.imageOrientation;
}

void MonoCamera::setResolution(MonoCameraProperties::SensorResolution resolution) {
    properties.resolution = resolution;
}

MonoCameraProperties::SensorResolution MonoCamera::getResolution() const {
    return properties.resolution;
}

void MonoCamera::setFrameEventFilter(const std::vector<dai::FrameEvent>& events) {
    properties.eventFilter = events;
}

std::vector<dai::FrameEvent> MonoCamera::getFrameEventFilter() const {
    return properties.eventFilter;
}

void MonoCamera::setFps(float fps) {
    properties.fps = fps;
}

void MonoCamera::setIsp3aFps(int isp3aFps) {
    properties.isp3aFps = isp3aFps;
}

float MonoCamera::getFps() const {
    // if AUTO
    if(properties.fps == -1 || properties.fps == 0) {
        return 30.0f;
    }

    // else return fps
    return properties.fps;
}

std::tuple<int, int> MonoCamera::getResolutionSize() const {
    switch(properties.resolution) {
        case MonoCameraProperties::SensorResolution::THE_400_P:
            return {640, 400};
            break;

        case MonoCameraProperties::SensorResolution::THE_720_P:
            return {1280, 720};
            break;

        case MonoCameraProperties::SensorResolution::THE_800_P:
            return {1280, 800};
            break;

        case MonoCameraProperties::SensorResolution::THE_480_P:
            return {640, 480};
            break;

        case MonoCameraProperties::SensorResolution::THE_1200_P:
            return {1920, 1200};
            break;
    }
    return {1280, 720};
}

int MonoCamera::getResolutionWidth() const {
    return std::get<0>(getResolutionSize());
}

int MonoCamera::getResolutionHeight() const {
    return std::get<1>(getResolutionSize());
}

void MonoCamera::setNumFramesPool(int num) {
    properties.numFramesPool = num;
}
void MonoCamera::setRawNumFramesPool(int num) {
    properties.numFramesPoolRaw = num;
}

int MonoCamera::getNumFramesPool() const {
    return properties.numFramesPool;
}
int MonoCamera::getRawNumFramesPool() const {
    return properties.numFramesPoolRaw;
}

void MonoCamera::setRawOutputPacked(bool packed) {
    properties.rawPacked = packed;
}

}  // namespace node
}  // namespace dai
