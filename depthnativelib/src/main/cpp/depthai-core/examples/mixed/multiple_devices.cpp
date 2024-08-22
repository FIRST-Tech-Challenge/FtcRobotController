#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

std::shared_ptr<dai::Pipeline> createPipeline() {
    // Start defining a pipeline
    auto pipeline = std::make_shared<dai::Pipeline>();
    // Define a source - color camera
    auto camRgb = pipeline->create<dai::node::ColorCamera>();

    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);

    // Create output
    auto xoutRgb = pipeline->create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    camRgb->preview.link(xoutRgb->input);

    return pipeline;
}

int main(int argc, char** argv) {
    auto deviceInfoVec = dai::Device::getAllAvailableDevices();
    const auto usbSpeed = dai::UsbSpeed::SUPER;
    auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;

    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> qRgbMap;
    std::vector<std::shared_ptr<dai::Device>> devices;

    for(auto& deviceInfo : deviceInfoVec) {
        auto device = std::make_shared<dai::Device>(openVinoVersion, deviceInfo, usbSpeed);
        devices.push_back(device);
        std::cout << "===Connected to " << deviceInfo.getMxId() << std::endl;
        auto mxId = device->getMxId();
        auto cameras = device->getConnectedCameras();
        auto usbSpeed = device->getUsbSpeed();
        auto eepromData = device->readCalibration2().getEepromData();
        std::cout << "   >>> MXID:" << mxId << std::endl;
        std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
        std::cout << "   >>> USB speed:" << usbSpeed << std::endl;
        if(eepromData.boardName != "") {
            std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
        }
        if(eepromData.productName != "") {
            std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
        }
        auto pipeline = createPipeline();
        device->startPipeline(*pipeline);

        auto qRgb = device->getOutputQueue("rgb", 4, false);
        std::string streamName = "rgb-" + eepromData.productName + mxId;
        qRgbMap.insert({streamName, qRgb});
    }
    while(true) {
        for(auto& element : qRgbMap) {
            auto qRgb = element.second;
            auto streamName = element.first;
            auto inRgb = qRgb->tryGet<dai::ImgFrame>();
            if(inRgb != nullptr) {
                cv::imshow(streamName, inRgb->getCvFrame());
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
