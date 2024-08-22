#include <csignal>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static std::atomic<bool> alive{true};
static void sigintHandler(int signum) {
    alive = false;
}

int main() {
    using namespace std;
    std::signal(SIGINT, &sigintHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");

    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setLeftRightCheck(false);
    stereo->setExtendedDisparity(false);
    // Subpixel disparity is of UINT16 format, which is unsupported by VideoEncoder
    stereo->setSubpixel(false);
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    videoEnc->setDefaultProfilePreset(monoLeft->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
    stereo->disparity.link(videoEnc->input);

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("disparity");
    videoEnc->bitstream.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the disparity frames from the outputs defined above
    auto q = device.getOutputQueue("disparity");

    auto videoFile = std::ofstream("disparity.mjpeg", std::ios::binary);
    cout << "Press Ctrl+C to stop encoding..." << endl;

    while(alive) {
        auto h265Packet = q->get<dai::ImgFrame>();
        videoFile.write((char*)(h265Packet->getData().data()), h265Packet->getData().size());
    }

    cout << "To view the encoded data, convert the stream file (.mjpeg) into a video file (.mp4) using a command below:" << endl;
    cout << "ffmpeg -framerate 30 -i disparity.mjpeg -c copy video.mp4" << endl;
    return 0;
}
