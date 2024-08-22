#include <chrono>
#include <iostream>
#include <thread>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Include nlohmann json
#include "nlohmann/json.hpp"

int main() {
    using namespace std;

    dai::Pipeline pipeline;

    auto xin = pipeline.create<dai::node::XLinkIn>();
    xin->setStreamName("in");

    auto script = pipeline.create<dai::node::Script>();
    xin->out.link(script->inputs["in"]);
    script->setScript(R"(
        import json

        # Receive bytes from the host
        data = node.io['in'].get().getData()
        jsonStr = str(data, 'utf-8')
        dict = json.loads(jsonStr)

        # Change initial dictionary a bit
        dict['one'] += 1
        dict['foo'] = "baz"

        b = Buffer(30)
        b.setData(json.dumps(dict).encode('utf-8'))
        node.io['out'].send(b)
    )");

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("out");
    script->outputs["out"].link(xout->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);

    // This dict will be serialized (JSON), sent to device (Script node),
    // edited a bit and sent back to the host
    nlohmann::json dict{{"one", 1}, {"foo", "bar"}};
    cout << "dict: " << dict << "\n";
    auto buffer = dai::Buffer();
    auto data = dict.dump();
    buffer.setData({data.begin(), data.end()});
    device.getInputQueue("in")->send(buffer);

    // Wait for the script to send the changed dictionary back
    auto jsonData = device.getOutputQueue("out")->get<dai::Buffer>();
    auto changedDict = nlohmann::json::parse(jsonData->getData());
    cout << "changedDict: " << changedDict << "\n";
    const nlohmann::json expectedDict{{"one", 2}, {"foo", "baz"}};
    if(expectedDict != changedDict) return 1;
    return 0;
}
