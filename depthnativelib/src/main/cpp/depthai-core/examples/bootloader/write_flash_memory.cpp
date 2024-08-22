#include <cstdio>
#include <fstream>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cout << "Usage: " << argv[0] << " offset filename" << std::endl;
        return 0;
    }
    size_t offset = std::stoi(argv[1]);
    std::string filename{argv[2]};

    // Find device and read memory
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.name << std::endl;
        dai::DeviceBootloader bl(info);

        auto progress = [](float p) { std::cout << "Flashing progress..." << p * 100 << "%" << std::endl; };
        bl.flashCustom(dai::DeviceBootloader::Memory::FLASH, offset, filename, progress);

    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}