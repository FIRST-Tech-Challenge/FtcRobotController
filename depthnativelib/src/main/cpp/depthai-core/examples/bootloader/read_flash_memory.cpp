#include <cstdio>
#include <fstream>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc < 3) {
        std::cout << "Usage: " << argv[0] << " offset size [filename]" << std::endl;
        return 0;
    }
    size_t offset = std::stoi(argv[1]);
    size_t size = std::stoi(argv[2]);
    std::string filename = "";
    if(argc == 4) {
        filename = std::string{argv[3]};
    }

    // Find device and read memory
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.name << std::endl;
        dai::DeviceBootloader bl(info);

        auto progress = [](float p) { std::cout << "Reading progress..." << p * 100 << "%" << std::endl; };

        if(filename.empty()) {
            std::vector<uint8_t> data;
            bl.readCustom(dai::DeviceBootloader::Memory::FLASH, offset, size, data, progress);

            size_t offset = 0;

            while(offset < data.size()) {
                // Print as hex, 32B aligned
                for(int i = 0; i < 32; i++) {
                    if(offset >= data.size()) {
                        break;
                    }
                    printf("%02X ", data[offset]);
                    offset++;
                }
                printf("\n");
            }

        } else {
            bl.readCustom(dai::DeviceBootloader::Memory::FLASH, offset, size, filename, progress);
        }

    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}