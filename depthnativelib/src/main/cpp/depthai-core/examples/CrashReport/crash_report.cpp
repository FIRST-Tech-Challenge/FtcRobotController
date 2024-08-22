#include <fstream>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static bool fileExists(dai::Path path) {
    std::ifstream file(path);
    return file.is_open();
}

int main() {
    using namespace std;

    // Connect to device and start pipeline
    dai::Device device;
    if(device.hasCrashDump()) {
        auto crashDump = device.getCrashDump();
        std::string commitHash = crashDump.depthaiCommitHash;
        std::string deviceId = crashDump.deviceId;

        auto json = crashDump.serializeToJson();

        for(int i = 0;; i++) {
            dai::Path destPath = "crashDump_" + to_string(i) + "_" + deviceId + "_" + commitHash + ".json";
            if(fileExists(destPath)) continue;

            std::ofstream ob(destPath);
            ob << std::setw(4) << json << std::endl;

            std::cout << "Crash dump found on your device!" << std::endl;
            std::cout << "Saved to " << destPath.string() << std::endl;
            std::cout << "Please report to developers!" << std::endl;
            break;
        }
    } else {
        std::cout << "There was no crash dump found on your device!" << std::endl;
    }

    return 0;
}