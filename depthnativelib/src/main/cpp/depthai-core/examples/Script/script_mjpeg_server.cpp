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
    cam->video.link(jpeg->input);

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setProcessor(dai::ProcessorType::LEON_CSS);
    jpeg->bitstream.link(script->inputs["jpeg"]);
    script->setScript(R"(
    import time
    import socket
    import fcntl
    import struct
    from socketserver import ThreadingMixIn
    from http.server import BaseHTTPRequestHandler, HTTPServer

    PORT = 8080

    def get_ip_address(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            -1071617759,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15].encode())
        )[20:24])

    class ThreadingSimpleServer(ThreadingMixIn, HTTPServer):
        pass

    class HTTPHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'<h1>[DepthAI] Hello, world!</h1><p>Click <a href="img">here</a> for an image</p>')
            elif self.path == '/img':
                try:
                    self.send_response(200)
                    self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
                    self.end_headers()
                    fpsCounter = 0
                    timeCounter = time.time()
                    while True:
                        jpegImage = node.io['jpeg'].get()
                        self.wfile.write("--jpgboundary".encode())
                        self.wfile.write(bytes([13, 10]))
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(jpegImage.getData())))
                        self.end_headers()
                        self.wfile.write(jpegImage.getData())
                        self.end_headers()

                        fpsCounter = fpsCounter + 1
                        if time.time() - timeCounter > 1:
                            node.warn(f'FPS: {fpsCounter}')
                            fpsCounter = 0
                            timeCounter = time.time()
                except Exception as ex:
                    node.warn(str(ex))

    with ThreadingSimpleServer(("", PORT), HTTPHandler) as httpd:
        node.warn(f"Serving at {get_ip_address('re0')}:{PORT}")
        httpd.serve_forever()
    )");

    // Connect to device with pipeline
    dai::Device device(pipeline);
    while(!device.isClosed()) {
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
    return 0;
}
