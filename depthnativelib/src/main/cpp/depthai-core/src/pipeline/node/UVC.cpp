#include "depthai/pipeline/node/UVC.hpp"

namespace dai {
namespace node {

UVC::UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : UVC(par, nodeId, std::make_unique<UVC::Properties>()) {}
UVC::UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, UVC, UVCProperties>(par, nodeId, std::move(props)) {
    setInputRefs(&input);
}

void UVC::setGpiosOnInit(std::unordered_map<int, int> list) {
    properties.gpioInit = list;
}

void UVC::setGpiosOnStreamOn(std::unordered_map<int, int> list) {
    properties.gpioStreamOn = list;
}

void UVC::setGpiosOnStreamOff(std::unordered_map<int, int> list) {
    properties.gpioStreamOff = list;
}

}  // namespace node
}  // namespace dai
