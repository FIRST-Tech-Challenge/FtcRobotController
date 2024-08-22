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
        import json
        data = json.dumps({
            'deviceId': __device_id__,
            'fwVersion': __version__
        }).encode('utf-8')

        b = Buffer(len(data))
        b.setData(data)
        node.io['info'].send(b)
    )");

    // XLinkOut
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("info");
    script->outputs["info"].link(xout->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);
    auto msg = device.getOutputQueue("info")->get<dai::Buffer>();
    auto data = nlohmann::json::parse(msg->getData());
    std::cout << data.dump(4) << std::endl;

    return 0;
}
