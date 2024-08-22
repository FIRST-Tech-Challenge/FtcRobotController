#pragma once

// std
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// shared
#include "depthai-bootloader-shared/Config.hpp"
#include "depthai-bootloader-shared/Memory.hpp"
#include "depthai-bootloader-shared/Section.hpp"
#include "depthai-bootloader-shared/Type.hpp"

namespace dai {

// DeviceBootloader (RAII), connects to device and maintains watchdog ...

/**
 * Represents the DepthAI bootloader with the methods to interact with it.
 */
class DeviceBootloader {
   public:
    // Alias
    using Type = dai::bootloader::Type;
    using Memory = dai::bootloader::Memory;
    using Section = dai::bootloader::Section;
    using UsbConfig = dai::bootloader::UsbConfig;
    using NetworkConfig = dai::bootloader::NetworkConfig;

    // Derive and extend bootloader::Config for easier usage
    struct Config : public bootloader::Config {
        /// Setting a static IPv4 won't start DHCP client
        void setStaticIPv4(std::string ip, std::string mask, std::string gateway);
        /// Setting a dynamic IPv4 will set that IP as well as start DHCP client
        void setDynamicIPv4(std::string ip, std::string mask, std::string gateway);
        /// Get if static IPv4 configuration is set
        bool isStaticIPV4();
        /// Get IPv4
        std::string getIPv4();
        /// Get IPv4 mask
        std::string getIPv4Mask();
        /// Get IPv4 gateway
        std::string getIPv4Gateway();
        /// Set IPv4 DNS options
        void setDnsIPv4(std::string dns, std::string dnsAlt = "");
        /// Get primary IPv4 DNS server
        std::string getDnsIPv4();
        /// Get alternate IPv4 DNS server
        std::string getDnsAltIPv4();

        /// Set USB timeout
        void setUsbTimeout(std::chrono::milliseconds ms);
        /// Get USB timeout
        std::chrono::milliseconds getUsbTimeout();

        /// Set NETWOR timeout
        void setNetworkTimeout(std::chrono::milliseconds ms);
        /// Get NETWORK timeout
        std::chrono::milliseconds getNetworkTimeout();

        /// Set MAC address if not flashed on controller
        void setMacAddress(std::string mac);
        /// Get MAC address if not flashed on controller
        std::string getMacAddress();

        /// Set maxUsbSpeed
        void setUsbMaxSpeed(UsbSpeed speed);
        /// Get maxUsbSpeed
        UsbSpeed getUsbMaxSpeed();

        /// To JSON
        nlohmann::json toJson() const;

        /// from JSON
        static Config fromJson(nlohmann::json);

       private:
        nlohmann::json data;
    };

    // Add an alias to Version
    using Version = dai::Version;

    struct ApplicationInfo {
        Memory memory;
        bool hasApplication;
        std::string firmwareVersion;
        std::string applicationName;
    };

    struct MemoryInfo {
        bool available;
        std::int64_t size;
        std::string info;
    };

    // constants

    /// Default Bootloader type
    static constexpr const Type DEFAULT_TYPE{Type::USB};

    // Static API
    /**
     * Searches for connected devices in either UNBOOTED or BOOTLOADER states and returns first available.
     * @returns Tuple of boolean and DeviceInfo. If found boolean is true and DeviceInfo describes the device. Otherwise false
     */
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();

    /**
     * Searches for connected devices in either UNBOOTED or BOOTLOADER states.
     * @returns Vector of all found devices
     */
    static std::vector<DeviceInfo> getAllAvailableDevices();

    /**
     * Creates application package which can be flashed to depthai device.
     * @param pipeline Pipeline from which to create the application package
     * @param pathToCmd Optional path to custom device firmware
     * @param compress Optional boolean which specifies if contents should be compressed
     * @param applicationName Optional name the application that is flashed
     * @returns Depthai application package
     */
    static std::vector<uint8_t> createDepthaiApplicationPackage(
        const Pipeline& pipeline, const dai::Path& pathToCmd = {}, bool compress = false, std::string applicationName = "", bool checkChecksum = false);

    /**
     * Creates application package which can be flashed to depthai device.
     * @param pipeline Pipeline from which to create the application package
     * @param compress Specifies if contents should be compressed
     * @param applicationName Name the application that is flashed
     * @returns Depthai application package
     */
    static std::vector<uint8_t> createDepthaiApplicationPackage(const Pipeline& pipeline,
                                                                bool compress,
                                                                std::string applicationName = "",
                                                                bool checkChecksum = false);

    /**
     * Saves application package to a file which can be flashed to depthai device.
     * @param path Path where to save the application package
     * @param pipeline Pipeline from which to create the application package
     * @param pathToCmd Optional path to custom device firmware
     * @param compress Optional boolean which specifies if contents should be compressed
     * @param applicationName Optional name the application that is flashed
     */
    static void saveDepthaiApplicationPackage(const dai::Path& path,
                                              const Pipeline& pipeline,
                                              const dai::Path& pathToCmd = {},
                                              bool compress = false,
                                              std::string applicationName = "",
                                              bool checkChecksum = false);

    /**
     * Saves application package to a file which can be flashed to depthai device.
     * @param path Path where to save the application package
     * @param pipeline Pipeline from which to create the application package
     * @param compress Specifies if contents should be compressed
     * @param applicationName Optional name the application that is flashed
     */
    static void saveDepthaiApplicationPackage(
        const dai::Path& path, const Pipeline& pipeline, bool compress, std::string applicationName = "", bool checkChecksum = false);

    /**
     * @returns Embedded bootloader version
     */
    static Version getEmbeddedBootloaderVersion();

    /**
     * @returns Embedded bootloader binary
     */
    static std::vector<std::uint8_t> getEmbeddedBootloaderBinary(Type type = DEFAULT_TYPE);

    DeviceBootloader() = delete;

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state; flashing not allowed
     * @param devInfo DeviceInfo of which to boot or connect to
     */
    explicit DeviceBootloader(const DeviceInfo& devInfo);

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state.
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param allowFlashingBootloader (bool) Set to true to allow flashing the devices bootloader
     */
    template <typename T, std::enable_if_t<std::is_same<T, bool>::value, bool> = true>
    DeviceBootloader(const DeviceInfo& devInfo, T allowFlashingBootloader);

    /**
     * Connects to device in bootloader of specified type. Throws if it wasn't possible.
     * This constructor will automatically boot into specified bootloader type if not already running
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param type Type of bootloader to boot/connect to.
     * @param allowFlashingBootloader Set to true to allow flashing the devices bootloader. Defaults to false
     */
    DeviceBootloader(const DeviceInfo& devInfo, Type type, bool allowFlashingBootloader = false);

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state with a custom bootloader firmware.
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param pathToBootloader Custom bootloader firmware to boot
     * @param allowFlashingBootloader Set to true to allow flashing the devices bootloader. Defaults to false
     */
    DeviceBootloader(const DeviceInfo& devInfo, const dai::Path& pathToBootloader, bool allowFlashingBootloader = false);

    /**
     * Connects to device with specified name/device id
     *
     * @param nameOrDeviceId Creates DeviceInfo with nameOrDeviceId to connect to
     * @param allowFlashingBootloader Set to true to allow flashing the devices bootloader. Defaults to false
     */
    DeviceBootloader(std::string nameOrDeviceId, bool allowFlashingBootloader = false);

    /**
     * @brief Destroy the Device Bootloader object
     *
     */
    ~DeviceBootloader();

    /**
     * Flashes a given pipeline to the device.
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param pipeline Pipeline to flash to the board
     * @param compress Compresses application to reduce needed memory size
     * @param applicationName Name the application that is flashed
     */
    std::tuple<bool, std::string> flash(std::function<void(float)> progressCallback,
                                        const Pipeline& pipeline,
                                        bool compress = false,
                                        std::string applicationName = "",
                                        Memory memory = Memory::AUTO,
                                        bool checkChecksum = false);

    /**
     * Flashes a given pipeline to the device.
     * @param pipeline Pipeline to flash to the board
     * @param compress Compresses application to reduce needed memory size
     * @param applicationName Optional name the application that is flashed
     */
    std::tuple<bool, std::string> flash(
        const Pipeline& pipeline, bool compress = false, std::string applicationName = "", Memory memory = Memory::AUTO, bool checkChecksum = false);

    /**
     * Reads information about flashed application in specified memory from device
     * @param memory Specifies which memory to query
     */
    ApplicationInfo readApplicationInfo(Memory memory);

    /**
     * Flashes a specific depthai application package that was generated using createDepthaiApplicationPackage or saveDepthaiApplicationPackage
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param package Depthai application package to flash to the board
     */
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::function<void(float)> progressCallback,
                                                                 std::vector<uint8_t> package,
                                                                 Memory memory = Memory::AUTO);

    /**
     * Flashes a specific depthai application package that was generated using createDepthaiApplicationPackage or saveDepthaiApplicationPackage
     * @param package Depthai application package to flash to the board
     */
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::vector<uint8_t> package, Memory memory = Memory::AUTO);

    /**
     * Clears flashed application on the device, by removing SBR boot structure
     * Doesn't remove fast boot header capability to still boot the application
     */
    std::tuple<bool, std::string> flashClear(Memory memory = Memory::AUTO);

    /**
     * Flashes bootloader to the current board
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param path Optional parameter to custom bootloader to flash
     */
    std::tuple<bool, std::string> flashBootloader(std::function<void(float)> progressCallback, const dai::Path& path = {});

    /**
     * Flash selected bootloader to the current board
     * @param memory Memory to flash
     * @param type Bootloader type to flash
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param path Optional parameter to custom bootloader to flash
     */
    std::tuple<bool, std::string> flashBootloader(Memory memory, Type type, std::function<void(float)> progressCallback, const dai::Path& path = {});

    /**
     * Flashes user bootloader to the current board. Available for NETWORK bootloader type
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param path Optional parameter to custom bootloader to flash
     */
    std::tuple<bool, std::string> flashUserBootloader(std::function<void(float)> progressCallback, const dai::Path& path = {});

    /**
     * Flash boot header which boots same as equivalent GPIO mode would
     * @param gpioMode GPIO mode equivalent
     */
    std::tuple<bool, std::string> flashGpioModeBootHeader(Memory memory, int gpioMode);

    /**
     * Flash USB recovery boot header. Switches to USB ROM Bootloader
     * @param memory Which memory to flash the header to
     */
    std::tuple<bool, std::string> flashUsbRecoveryBootHeader(Memory memory);

    /**
     * Flash optimized boot header
     * @param memory Which memory to flasht the header to
     * @param frequency SPI specific parameter, frequency in MHz
     * @param location Target location the header should boot to. Default to location of bootloader
     * @param dummyCycles SPI specific parameter
     * @param offset Offset in memory to flash the header to. Defaults to offset of boot header
     * @returns status as std::tuple<bool, std::string>
     */
    std::tuple<bool, std::string> flashBootHeader(Memory memory, int32_t frequency = -1, int64_t location = -1, int32_t dummyCycles = -1, int64_t offset = -1);

    /**
     * Flash fast boot header. Application must already be present in flash, or location must be specified manually.
     * Note - Can soft brick your device if firmware location changes.
     * @param memory Which memory to flash the header to
     * @param frequency SPI specific parameter, frequency in MHz
     * @param location Target location the header should boot to. Default to location of bootloader
     * @param dummyCycles SPI specific parameter
     * @param offset Offset in memory to flash the header to. Defaults to offset of boot header
     * @returns status as std::tuple<bool, std::string>
     */
    std::tuple<bool, std::string> flashFastBootHeader(
        Memory memory, int32_t frequency = -1, int64_t location = -1, int32_t dummyCycles = -1, int64_t offset = -1);

    /**
     * Flash arbitrary data at custom offset in specified memory
     * @param memory Memory to flash
     * @param offset Offset at which to flash the given data in bytes
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param data Data to flash
     */
    std::tuple<bool, std::string> flashCustom(Memory memory, size_t offset, const std::vector<uint8_t>& data, std::function<void(float)> progressCb = nullptr);
    std::tuple<bool, std::string> flashCustom(Memory memory, size_t offset, const uint8_t* data, size_t size, std::function<void(float)> progressCb = nullptr);
    std::tuple<bool, std::string> flashCustom(Memory memory, size_t offset, std::string filename, std::function<void(float)> progressCb = nullptr);

    /**
     * Reads arbitrary data at custom offset in specified memory
     * @param memory Memory to read
     * @param offset Offset at which to read the specified bytes
     * @param size Number of bytes to read
     * @param data Data to read to. Must be at least 'size' number of bytes big
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current reading progress
     */
    std::tuple<bool, std::string> readCustom(
        Memory memory, size_t offset, size_t size, std::vector<uint8_t>& data, std::function<void(float)> progressCb = nullptr);
    std::tuple<bool, std::string> readCustom(Memory memory, size_t offset, size_t size, uint8_t* data, std::function<void(float)> progressCb = nullptr);
    std::tuple<bool, std::string> readCustom(Memory memory, size_t offset, size_t size, std::string filename, std::function<void(float)> progressCb = nullptr);
    std::tuple<bool, std::string, std::vector<uint8_t>> readCustom(Memory memory, size_t offset, size_t size, std::function<void(float)> progressCb = nullptr);

    /**
     * Reads configuration data from bootloader
     * @returns Unstructured configuration data
     * @param memory Optional - from which memory to read configuration data
     * @param type Optional - from which type of bootloader to read configuration data
     */
    nlohmann::json readConfigData(Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Flashes configuration data to bootloader
     * @param configData Unstructured configuration data
     * @param memory Optional - to which memory flash configuration
     * @param type Optional - for which type of bootloader to flash configuration
     */
    std::tuple<bool, std::string> flashConfigData(nlohmann::json configData, Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Flashes configuration data to bootloader
     * @param configPath Unstructured configuration data
     * @param memory Optional - to which memory flash configuration
     * @param type Optional - for which type of bootloader to flash configuration
     */
    std::tuple<bool, std::string> flashConfigFile(const dai::Path& configPath, Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Clears configuration data
     * @param memory Optional - on which memory to clear configuration data
     * @param type Optional - for which type of bootloader to clear configuration data
     */
    std::tuple<bool, std::string> flashConfigClear(Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Reads configuration from bootloader
     * @param memory Optional - from which memory to read configuration
     * @param type Optional - from which type of bootloader to read configuration
     * @returns Configuration structure
     */
    Config readConfig(Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Flashes configuration to bootloader
     * @param configData Configuration structure
     * @param memory Optional - to which memory flash configuration
     * @param type Optional - for which type of bootloader to flash configuration
     */
    std::tuple<bool, std::string> flashConfig(const Config& config, Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Retrieves information about specified memory
     * @param memory Specifies which memory to query
     */
    MemoryInfo getMemoryInfo(Memory memory);

    /**
     * Checks whether User Bootloader is supported with current bootloader
     * @returns true of User Bootloader is supported, false otherwise
     */
    bool isUserBootloaderSupported();

    /**
     * Retrieves whether current bootloader is User Bootloader (B out of A/B configuration)
     */
    bool isUserBootloader();

    /**
     * Boots a custom FW in memory
     * @param fw
     * @throws A runtime exception if there are any communication issues
     */
    void bootMemory(const std::vector<uint8_t>& fw);

    /**
     * Boots into integrated ROM bootloader in USB mode
     * @throws A runtime exception if there are any communication issues
     */
    void bootUsbRomBootloader();

    /**
     * @returns Version of current running bootloader
     */
    Version getVersion() const;

    /**
     * @returns True when bootloader was booted using latest bootloader integrated in the library.
     * False when bootloader is already running on the device and just connected to.
     */
    bool isEmbeddedVersion() const;

    /**
     * @returns Type of currently connected bootloader
     */
    Type getType() const;

    /**
     * @returns True if allowed to flash bootloader
     */
    bool isAllowedFlashingBootloader() const;

    /**
     * Explicitly closes connection to device.
     * @note This function does not need to be explicitly called
     * as destructor closes the device automatically
     */
    void close();

    /**
     * Is the device already closed (or disconnected)
     *
     * @warning This function is thread-unsafe and may return outdated incorrect values. It is
     * only meant for use in simple single-threaded code. Well written code should handle
     * exceptions when calling any DepthAI apis to handle hardware events and multithreaded use.
     */
    bool isClosed() const;

   private:
    // private static

    // private methods
    void init(bool embeddedMvcmd, const dai::Path& pathToMvcmd, tl::optional<bootloader::Type> type, bool allowBlFlash);
    template <typename T>
    bool sendRequest(const T& request);
    template <typename T>
    void sendRequestThrow(const T& request);
    bool receiveResponseData(std::vector<uint8_t>& data);
    template <typename T>
    bool parseResponse(const std::vector<uint8_t>& data, T& response);
    template <typename T>
    bool receiveResponse(T& response);
    template <typename T>
    void receiveResponseThrow(T& response);
    Version requestVersion();
    std::tuple<bool, std::string> flashCustom(
        Memory memory, size_t offset, const uint8_t* data, size_t size, std::string filename, std::function<void(float)> progressCb);
    std::tuple<bool, std::string> readCustom(
        Memory memory, size_t offset, size_t size, uint8_t* data, std::string filename, std::function<void(float)> progressCb);

    // private variables
    std::shared_ptr<XLinkConnection> connection;
    DeviceInfo deviceInfo = {};

    bool isEmbedded = false;
    Type bootloaderType;

    // closed
    std::atomic<bool> closed{false};

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // Monitor thread
    std::thread monitorThread;
    std::mutex lastWatchdogPingTimeMtx;
    std::chrono::steady_clock::time_point lastWatchdogPingTime;

    // bootloader stream
    std::unique_ptr<XLinkStream> stream;

    // Allow flashing bootloader flag
    bool allowFlashingBootloader = false;

    // Current connected bootloader version
    Version version{0, 0, 2};
};

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Type& type) {
    switch(type) {
        case dai::DeviceBootloader::Type::AUTO:
            out << "AUTO";
            break;
        case dai::DeviceBootloader::Type::USB:
            out << "USB";
            break;
        case dai::DeviceBootloader::Type::NETWORK:
            out << "NETWORK";
            break;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Memory& memory) {
    switch(memory) {
        case dai::DeviceBootloader::Memory::AUTO:
            out << "AUTO";
            break;
        case dai::DeviceBootloader::Memory::FLASH:
            out << "FLASH";
            break;
        case dai::DeviceBootloader::Memory::EMMC:
            out << "EMMC";
            break;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Section& type) {
    switch(type) {
        case dai::DeviceBootloader::Section::AUTO:
            out << "AUTO";
            break;
        case dai::DeviceBootloader::Section::HEADER:
            out << "HEADER";
            break;
        case dai::DeviceBootloader::Section::BOOTLOADER:
            out << "BOOTLOADER";
            break;
        case dai::DeviceBootloader::Section::USER_BOOTLOADER:
            out << "USER_BOOTLOADER";
            break;
        case dai::DeviceBootloader::Section::BOOTLOADER_CONFIG:
            out << "BOOTLOADER_CONFIG";
            break;
        case dai::DeviceBootloader::Section::APPLICATION:
            out << "APPLICATION";
            break;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Version& v) {
    return out << v.toString();
}