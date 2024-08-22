#include <fstream>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono_literals;

    // Create a log file
    ofstream logstream{"device.log"};

    // Connect to device and start pipeline
    dai::Device::Config config;
    config.logLevel = dai::LogLevel::DEBUG;
    config.outputLogLevel = dai::LogLevel::WARN;
    config.board.logDevicePrints = true;
    dai::Device device(config, dai::UsbSpeed::SUPER);

    // Configure logging
    device.addLogCallback([&logstream](dai::LogMessage msg) {
        logstream << "[" << msg.nodeIdName << "] " << msg.payload << endl;
        cout << "[LOGGED] [" << msg.nodeIdName << "] " << msg.payload << endl;
    });

    // Print available sensors
    cout << "Available camera sensors: ";
    for(auto& sensor : device.getCameraSensorNames()) {
        cout << "Socket: " << sensor.first << " - " << sensor.second << ", ";
    }
    cout << endl;

    // Print device name
    cout << "Device name: " << device.getDeviceName() << endl;

    // Wait a tad for some logs to arrive
    this_thread::sleep_for(3s);

    return 0;
}
