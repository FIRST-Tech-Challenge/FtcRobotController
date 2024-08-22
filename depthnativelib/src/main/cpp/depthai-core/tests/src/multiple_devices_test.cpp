#include <atomic>
#include <iostream>
#include <tuple>
#include <vector>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

constexpr auto NUM_MESSAGES = 50;
constexpr auto TEST_TIMEOUT = 20s;

int main() {
    mutex mtx;
    vector<thread> threads;
    vector<tuple<shared_ptr<dai::Device>, int>> devices;
    int deviceCounter = 0;

    // Wait for 3s to acquire more than 1 device. Otherwise perform the test with 1 device
    // Could also fail instead - but requires test groups before
    auto t1 = steady_clock::now();
    vector<dai::DeviceInfo> availableDevices;
    do {
        availableDevices = dai::Device::getAllAvailableDevices();
        this_thread::sleep_for(500ms);
    } while(availableDevices.size() <= 1 && steady_clock::now() - t1 <= 3s);

    for(const auto& dev : availableDevices) {
        threads.push_back(thread([&mtx, &devices, dev, deviceCounter]() {
            // Create pipeline
            dai::Pipeline pipeline;

            // Define source and output
            auto camRgb = pipeline.create<dai::node::ColorCamera>();
            auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

            xoutRgb->setStreamName("rgb");

            // Properties
            camRgb->setPreviewSize(300, 300);
            camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
            camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            camRgb->setInterleaved(false);
            camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

            // Linking
            camRgb->preview.link(xoutRgb->input);

            // Optional delay between device connection
            // if(deviceCounter) this_thread::sleep_for(1s);
            std::ignore = deviceCounter;

            auto device = make_shared<dai::Device>(pipeline, dev, dai::UsbSpeed::SUPER);
            device->getOutputQueue("rgb", 4, false);

            cout << "MXID: " << device->getMxId() << endl;
            cout << "Connected cameras: ";
            for(const auto& cam : device->getConnectedCameras()) {
                cout << cam << " ";
            }
            cout << endl;

            unique_lock<std::mutex> l(mtx);
            devices.push_back({device, 0});
        }));

        deviceCounter++;
    }

    // Join device threads
    for(auto& thread : threads) {
        thread.join();
    }

    bool finished = false;
    t1 = steady_clock::now();
    do {
        {
            std::unique_lock<std::mutex> l(mtx);

            finished = devices.size() > 0;
            for(auto& devCounter : devices) {
                auto& dev = get<0>(devCounter);
                auto& counter = get<1>(devCounter);
                if(dev->getOutputQueue("rgb")->tryGet<dai::ImgFrame>()) {
                    cout << "Device " << dev->getMxId() << " message arrived (" << counter + 1 << "/" << NUM_MESSAGES << ")\n";
                    counter++;
                }

                if(counter < NUM_MESSAGES) {
                    finished = false;
                }
            }
        }

        std::this_thread::sleep_for(1ms);
    } while(!finished && steady_clock::now() - t1 < TEST_TIMEOUT);

    return finished == 0;
}
