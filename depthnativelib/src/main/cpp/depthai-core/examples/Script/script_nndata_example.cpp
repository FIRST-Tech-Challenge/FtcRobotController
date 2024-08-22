#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    buf = NNData(150)
    buf.setLayer("fp16", [1.0, 1.2, 3.9, 5.5])
    buf.setLayer("uint8", [6, 9, 4, 2, 0])
    node.info("Names of layers: " + str(buf.getAllLayerNames()))
    node.io['host'].send(buf)
    )");

    // XLinkOut
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("host");
    script->outputs["host"].link(xout->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);

    device.setLogLevel(dai::LogLevel::WARN);
    device.setLogOutputLevel(dai::LogLevel::WARN);

    auto nndata = device.getOutputQueue("host")->get<dai::NNData>();

    std::cout << "NNData size: " << nndata->getData().size() << std::endl;

    std::cout << "FP16 values: ";
    for(auto val : nndata->getLayerFp16("fp16")) {
        std::cout << to_string(val) + " ";
    }
    std::cout << std::endl;

    std::cout << "UINT8 values: ";
    for(auto val : nndata->getLayerUInt8("uint8")) {
        std::cout << to_string(val) + " ";
    }
    std::cout << std::endl;
    return 0;
}
