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

int main(int argc, char** argv) {
    using namespace std::chrono;

    dai::DeviceBootloader::Type blType = dai::DeviceBootloader::Type::AUTO;
    if(argc > 1) {
        std::string blTypeStr(argv[1]);
        if(blTypeStr == "usb") {
            blType = dai::DeviceBootloader::Type::USB;
        } else if(blTypeStr == "network") {
            blType = dai::DeviceBootloader::Type::NETWORK;
        } else {
            std::cout << "Specify either 'usb' or 'network' bootloader type\n";
            return 0;
        }
    }

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
        std::cout << "Which DepthAI device to flash bootloader for [0.." << deviceInfos.size() - 1 << "]\n";
        std::cin >> selected;
        info = deviceInfos[selected];
    }

    bool hasBootloader = (info.state == X_LINK_BOOTLOADER);
    if(hasBootloader) {
        std::cout << "Warning! Flashing bootloader can potentially soft brick your device and should be done with caution." << std::endl;
        std::cout << "Do not unplug your device while the bootloader is flashing." << std::endl;
        std::cout << "Type 'y' and press enter to proceed, otherwise exits: ";
        std::cin.ignore();
        if(std::cin.get() != 'y') {
            std::cout << "Prompt declined, exiting..." << std::endl;
            return -1;
        }
    }

    // Open DeviceBootloader and allow flashing bootloader
    std::cout << "Booting latest bootloader first, will take a tad longer..." << std::endl;
    dai::DeviceBootloader bl(info, true);
    auto currentBlType = bl.getType();

    if(blType == dai::DeviceBootloader::Type::AUTO) {
        blType = currentBlType;
    }

    // Check if bootloader type is the same, if already booted by bootloader (not in USB recovery mode)
    if(currentBlType != blType && hasBootloader) {
        std::cout << "Are you sure you want to flash '" << blType << "' bootloader over current '" << currentBlType << "' bootloader?" << std::endl;
        std::cout << "Type 'y' and press enter to proceed, otherwise exits: ";
        std::cin.ignore();
        if(std::cin.get() != 'y') {
            std::cout << "Prompt declined, exiting..." << std::endl;
            return -1;
        }
    }

    // Create a progress callback lambda
    auto progress = [](float p) { std::cout << "Flashing Progress..." << p * 100 << "%" << std::endl; };

    std::cout << "Flashing " << blType << " bootloader..." << std::endl;
    auto t1 = steady_clock::now();
    bool success = false;
    std::string message;
    std::tie(success, message) = bl.flashBootloader(dai::DeviceBootloader::Memory::FLASH, blType, progress);
    if(success) {
        std::cout << "Flashing successful. Took " << duration_cast<milliseconds>(steady_clock::now() - t1).count() << "ms" << std::endl;
    } else {
        std::cout << "Flashing failed: " << message << std::endl;
    }
    return 0;
}