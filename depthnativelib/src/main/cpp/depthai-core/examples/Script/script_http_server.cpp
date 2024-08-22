#include <chrono>
#include <iostream>
#include <thread>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::ColorCamera>();

    auto jpeg = pipeline.create<dai::node::VideoEncoder>();
    jpeg->setDefaultProfilePreset(cam->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setProcessor(dai::ProcessorType::LEON_CSS);
    script->setScript(R"(
    from http.server import BaseHTTPRequestHandler
    import socketserver
    import socket
    import fcntl
    import struct

    PORT = 8080
    ctrl = CameraControl()
    ctrl.setCaptureStill(True)

    def get_ip_address(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            -1071617759,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode())
        )[20:24])

    class HTTPHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'<h1>[DepthAI] Hello, world!</h1><p>Click <a href="img">here</a> for an image</p>')
            elif self.path == '/img':
                node.io['out'].send(ctrl)
                jpegImage = node.io['jpeg'].get()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpegImage.getData())))
                self.end_headers()
                self.wfile.write(jpegImage.getData())
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b'Url not found...')

    with socketserver.TCPServer(("", PORT), HTTPHandler) as httpd:
        node.warn(f"Serving at {get_ip_address('re0')}:{PORT}")
        httpd.serve_forever()
    )");

    cam->still.link(jpeg->input);
    script->outputs["out"].link(cam->inputControl);
    jpeg->bitstream.link(script->inputs["jpeg"]);

    // Connect to device with pipeline
    dai::Device device(pipeline);
    while(!device.isClosed()) {
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
    return 0;
}
