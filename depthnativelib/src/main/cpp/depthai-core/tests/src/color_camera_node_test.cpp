#include <chrono>

#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;

    auto colorCam = p.create<dai::node::ColorCamera>();
    auto xout = p.create<dai::node::XLinkOut>();
    // auto xout2 = p.create<dai::node::XLinkOut>();

    const int previewWidth = 640, previewHeight = 640;
    // const int videoWidth = 1280, videoHeight = 1080;

    // XLinkOut
    xout->setStreamName("preview");
    // xout2->setStreamName("video");

    // ColorCamera
    colorCam->setPreviewSize(previewWidth, previewHeight);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xout->input);
    // colorCam->video.link(xout2->input);

    // Wait 5 seconds for device
    auto wait = steady_clock::now();
    bool found;
    dai::DeviceInfo deviceInfo;

    do {
        std::tie(found, deviceInfo) = dai::Device::getFirstAvailableDevice();

        if(found) {
            dai::Device d(p, deviceInfo);

            auto previewQueue = d.getOutputQueue("preview", 8, true);

            // Retrieve 10 messages
            for(size_t i = 0; i < 10; i++) {
                auto t1 = std::chrono::steady_clock::now();
                auto preview = previewQueue->get<dai::ImgFrame>();

                // check that the received imgframe has width & height of previewWidth & previewHeight
                if(preview->getWidth() != previewWidth || preview->getHeight() != previewHeight) {
                    return -1;
                }
            }

            // Exit with success error code
            return 0;
        }

        this_thread::sleep_for(1s);
    } while(!found && steady_clock::now() - wait < 5s);

    // Otherwise exit with error
    return -1;
}