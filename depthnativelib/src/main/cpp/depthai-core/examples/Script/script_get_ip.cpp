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
    import socket
    import fcntl
    import struct

    def get_ip_address(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            -1071617759,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode())
        )[20:24])

    ip = get_ip_address('re0')  # '192.168.0.110'
    node.warn(f'IP of the device: {ip}')
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
