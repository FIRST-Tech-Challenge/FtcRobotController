#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setProcessor(dai::ProcessorType::LEON_CSS);
    script->setScript(R"(
    import http.client
    import time

    node.warn('Sending http GET request...')
    h1 = http.client.HTTPConnection('api.ipify.org', 80)
    h1.request("GET", "/")
    r1 = h1.getresponse()
    node.warn(f'{r1.status} {r1.reason}')
    data1 = r1.read()
    node.warn(f'Public IP: {data1}')

    node.io['end'].send(Buffer(32))
    )");

    // XLinkOut
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("end");
    script->outputs["end"].link(xout->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);
    device.getOutputQueue("end")->get<dai::Buffer>();
    return 0;
}
