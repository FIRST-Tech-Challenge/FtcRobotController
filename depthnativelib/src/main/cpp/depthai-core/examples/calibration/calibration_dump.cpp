#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    dai::Device device(dai::OpenVINO::VERSION_2021_4, dai::UsbSpeed::HIGH);

    std::cout << "Is EEPROM available: " << device.isEepromAvailable() << std::endl;

    try {
        nlohmann::json j = device.readCalibration2().eepromToJson();
        std::cout << "User calibration: " << j.dump(4) << std::endl << std::endl;
    } catch(const std::exception& ex) {
        std::cout << "No user calibration: " << ex.what() << std::endl;
    }

    try {
        nlohmann::json j = device.readFactoryCalibration().eepromToJson();
        std::cout << "Factory calibration: " << j.dump(4) << std::endl << std::endl;
    } catch(const std::exception& ex) {
        std::cout << "No factory calibration: " << ex.what() << std::endl;
    }

    {
        nlohmann::json j = device.readCalibrationRaw();
        std::cout << "User calibration raw: " << j.dump() << std::endl << std::endl;
    }
    {
        nlohmann::json j = device.readFactoryCalibrationRaw();
        std::cout << "Factory calibration raw: " << j.dump() << std::endl << std::endl;
    }

    return 0;
}
