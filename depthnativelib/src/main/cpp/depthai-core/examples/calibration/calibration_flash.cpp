#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibJsonFile(CALIB_PATH);
    std::string calibBackUpFile("depthai_calib_backup.json");
    if(argc > 1) {
        calibJsonFile = std::string(argv[1]);
    }

    // Connect device
    dai::Device device(dai::OpenVINO::VERSION_UNIVERSAL, dai::UsbSpeed::HIGH);

    dai::CalibrationHandler deviceCalib = device.readCalibration();
    deviceCalib.eepromToJsonFile(calibBackUpFile);
    std::cout << "Calibration Data on the device is backed up at:" << calibBackUpFile << std::endl;
    dai::CalibrationHandler calibData(calibJsonFile);

    try {
        device.flashCalibration2(calibData);
        std::cout << "Successfully flashed calibration" << std::endl;
    } catch(const std::exception& ex) {
        std::cout << "Failed flashing calibration: " << ex.what() << std::endl;
    }

    return 0;
}
