#pragma once

#include <mutex>
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <thread>

// project
#include <depthai/device/Device.hpp>
#include <depthai/device/DeviceBootloader.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <depthai/utility/Path.hpp>

namespace dai {

class Resources {
    // private constructor
    Resources();
    ~Resources();

    mutable std::mutex mtxDevice;
    mutable std::condition_variable cvDevice;
    std::thread lazyThreadDevice;
    bool readyDevice;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMapDevice;

    mutable std::mutex mtxBootloader;
    mutable std::condition_variable cvBootloader;
    std::thread lazyThreadBootloader;
    bool readyBootloader;
    std::unordered_map<std::string, std::vector<std::uint8_t>> resourceMapBootloader;

public:
    static Resources& getInstance();
    Resources(Resources const&) = delete;
    void operator=(Resources const&) = delete;

    // Available resources
    std::vector<std::uint8_t> getDeviceFirmware(bool usb2Mode, OpenVINO::Version version = OpenVINO::VERSION_UNIVERSAL) const;
    std::vector<std::uint8_t> getDeviceFirmware(Device::Config config, dai::Path pathToMvcmd = {}) const;
    std::vector<std::uint8_t> getBootloaderFirmware(DeviceBootloader::Type type = DeviceBootloader::Type::USB) const;

};

} // namespace dai
