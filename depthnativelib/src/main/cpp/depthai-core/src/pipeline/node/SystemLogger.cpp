#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

SystemLogger::SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : SystemLogger(par, nodeId, std::make_unique<SystemLogger::Properties>()) {}
SystemLogger::SystemLogger(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, SystemLogger, SystemLoggerProperties>(par, nodeId, std::move(props)) {
    properties.rateHz = 1.0f;

    setOutputRefs(&out);
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

float SystemLogger::getRate() {
    return properties.rateHz;
}

}  // namespace node
}  // namespace dai
