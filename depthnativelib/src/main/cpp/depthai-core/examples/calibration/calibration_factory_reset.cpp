#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    dai::Device device(dai::OpenVINO::VERSION_2021_4, dai::UsbSpeed::HIGH);

    try {
        device.factoryResetCalibration();
        std::cout << "Factory reset calibration OK" << std::endl;
    } catch(const std::exception& ex) {
        std::cout << "Factory reset calibration FAIL: " << ex.what() << std::endl;
    }

    return 0;
}
