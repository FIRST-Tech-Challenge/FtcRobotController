#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    std::cout << "Searching for all available devices...\n\n";
    auto infos = dai::Device::getAllAvailableDevices();

    if(infos.size() <= 0) {
        std::cout << "Couldn't find any available devices.\n";
        return -1;
    }

    for(auto& info : infos) {
        std::cout << "Found device: " << info.name << " mxid: " << info.mxid << " state: " << info.state << std::endl;
    }

    // Connect to device and start pipeline
    std::cout << "\nBooting the first available camera (" << infos[0].name << ")...\n";
    dai::Device device(dai::Pipeline(), infos[0]);
    std::cout << "Available camera sensors: ";
    for(auto& sensor : device.getCameraSensorNames()) {
        std::cout << "Socket: " << sensor.first << " - " << sensor.second << ", ";
    }
    std::cout << std::endl;

    auto eeprom = device.readCalibration2().getEepromData();
    std::cout << "Product name: " << eeprom.productName << ", board name: " << eeprom.boardName << std::endl;

    return 0;
}
