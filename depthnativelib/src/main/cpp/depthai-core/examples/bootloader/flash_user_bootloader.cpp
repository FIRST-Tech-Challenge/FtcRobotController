#include <chrono>
#include <string>

#include "XLink/XLink.h"
#include "depthai/depthai.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

static const char* ProtocolToStr(XLinkProtocol_t val) {
    switch(val) {
        case X_LINK_USB_VSC:
            return "X_LINK_USB_VSC";
        case X_LINK_USB_CDC:
            return "X_LINK_USB_CDC";
        case X_LINK_PCIE:
            return "X_LINK_PCIE";
        case X_LINK_IPC:
            return "X_LINK_IPC";
        case X_LINK_TCP_IP:
            return "X_LINK_TCP_IP";
        case X_LINK_NMB_OF_PROTOCOLS:
            return "X_LINK_NMB_OF_PROTOCOLS";
        case X_LINK_ANY_PROTOCOL:
            return "X_LINK_ANY_PROTOCOL";
        default:
            return "INVALID_ENUM_VALUE";
            break;
    }
}

int main(int argc, char** argv) try {
    using namespace std::chrono;

    dai::DeviceInfo info;
    auto deviceInfos = dai::DeviceBootloader::getAllAvailableDevices();
    if(deviceInfos.empty()) {
        std::cout << "No device found to flash. Exiting." << std::endl;
        return -1;
    } else {
        for(int i = 0; i < deviceInfos.size(); i++) {
            const auto& devInfo = deviceInfos[i];
            std::cout << "[" << i << "] " << devInfo.getMxId() << "[" << ProtocolToStr(devInfo.protocol) << "]";
            if(devInfo.state == X_LINK_BOOTLOADER) {
                dai::DeviceBootloader bl(devInfo);
                std::cout << " current bootloader: " << bl.getVersion();
            }
            std::cout << std::endl;
        }
        int selected = 0;
        std::cout << "Which DepthAI device to flash bootloader for (Note: Only NETWORK supported) [0.." << deviceInfos.size() - 1 << "]\n";
        std::cin >> selected;
        info = deviceInfos[selected];
    }

    dai::DeviceBootloader bl(info);

    // Create a progress callback lambda
    auto progress = [](float p) { std::cout << "Flashing Progress..." << p * 100 << "%" << std::endl; };

    std::cout << "Flashing User Bootloader..." << std::endl;
    auto t1 = steady_clock::now();
    bool success = false;
    std::string message;
    std::tie(success, message) = bl.flashUserBootloader(progress);
    if(success) {
        std::cout << "Flashing successful. Took " << duration_cast<milliseconds>(steady_clock::now() - t1).count() << "ms" << std::endl;
    } else {
        std::cout << "Flashing failed: " << message << std::endl;
    }
    return 0;
} catch(const std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
}