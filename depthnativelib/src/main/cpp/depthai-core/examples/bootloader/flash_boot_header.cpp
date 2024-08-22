#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cout << "Usage: " << argv[0] << " [GPIO_MODE/USB_RECOVERY/NORMAL/FAST] [params]" << std::endl;
        std::cout << "\tOptions:\n";
        std::cout << "\t\t" << argv[0] << " GPIO_MODE gpioModeNum" << std::endl;
        std::cout << "\t\t" << argv[0] << " USB_RECOVERY" << std::endl;
        std::cout << "\t\t" << argv[0] << " NORMAL [frequency] [location] [dummyCycles] [offset]" << std::endl;
        std::cout << "\t\t" << argv[0] << " FAST [frequency] [location] [dummyCycles] [offset]" << std::endl;
        return 0;
    }
    std::string mode{argv[1]};
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    std::function<std::tuple<bool, std::string>(dai::DeviceBootloader&)> flash = nullptr;

    if(mode == "gpio_mode") {
        // gpio mode header
        if(argc < 3) {
            std::cout << "Usage: " << argv[0] << " GPIO_MODE gpioModeNum" << std::endl;
            return 0;
        }
        int gpioMode = std::stoi(std::string(argv[2]));

        flash = [gpioMode](dai::DeviceBootloader& bl) { return bl.flashGpioModeBootHeader(dai::DeviceBootloader::Memory::FLASH, gpioMode); };
    } else if(mode == "usb_recovery") {
        // usb recovery header
        flash = [](dai::DeviceBootloader& bl) { return bl.flashUsbRecoveryBootHeader(dai::DeviceBootloader::Memory::FLASH); };
    } else if(mode == "normal" || mode == "fast") {
        if(argc != 2 && argc != 3 && argc <= 3) {
            std::cout << "Usage: " << argv[0] << " NORMAL/FAST [frequency] [location] [dummyCycles] [offset]" << std::endl;
            std::cout << "Usage: " << argv[0] << " NORMAL/FAST [frequency]" << std::endl;
            return 0;
        }
        int64_t offset = -1;
        int64_t location = -1;
        int32_t dummyCycles = -1;
        int32_t frequency = -1;
        if(argc > 3) {
            if(argc >= 3) {
                offset = std::stoi(std::string(argv[2]));
            }
            if(argc >= 4) {
                location = std::stoi(std::string(argv[3]));
            }
            if(argc >= 5) {
                dummyCycles = std::stoi(std::string(argv[4]));
            }
            if(argc >= 6) {
                frequency = std::stoi(std::string(argv[5]));
            }
        } else if(argc == 3) {
            frequency = std::stoi(std::string(argv[2]));
        }

        if(mode == "normal") {
            flash = [offset, location, dummyCycles, frequency](dai::DeviceBootloader& bl) {
                return bl.flashBootHeader(dai::DeviceBootloader::Memory::FLASH, frequency, location, dummyCycles, offset);
            };
        } else if(mode == "fast") {
            flash = [offset, location, dummyCycles, frequency](dai::DeviceBootloader& bl) {
                return bl.flashFastBootHeader(dai::DeviceBootloader::Memory::FLASH, frequency, location, dummyCycles, offset);
            };
        }
    }

    // Find device and flash specified boot header
    bool res = false;
    dai::DeviceInfo info;
    std::tie(res, info) = dai::DeviceBootloader::getFirstAvailableDevice();

    if(res) {
        std::cout << "Found device with name: " << info.name << std::endl;
        dai::DeviceBootloader bl(info);
        // Flash the specified boot header
        if(flash) {
            bool success;
            std::string errorMsg;
            std::tie(success, errorMsg) = flash(bl);
            if(success) {
                std::cout << "Successfully flashed boot header!" << std::endl;
            } else {
                std::cout << "Couldn't flash boot header: " << errorMsg << std::endl;
            }
        } else {
            std::cout << "Invalid boot option header specified" << std::endl;
        }
    } else {
        std::cout << "No devices found" << std::endl;
    }

    return 0;
}