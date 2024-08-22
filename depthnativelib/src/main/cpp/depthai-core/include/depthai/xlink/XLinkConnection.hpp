#pragma once

// Std
#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

// project
#include "depthai/utility/Path.hpp"
#include "depthai/utility/ProfilingData.hpp"

// Libraries
#include <XLink/XLinkPublicDefines.h>

namespace dai {

/**
 * Describes a connected device
 */
struct DeviceInfo {
    DeviceInfo() = default;
    DeviceInfo(std::string name, std::string mxid, XLinkDeviceState_t state, XLinkProtocol_t protocol, XLinkPlatform_t platform, XLinkError_t status);
    /**
     * Creates a DeviceInfo by checking whether supplied parameter is a MXID or IP/USB name
     * @param mxidOrName Either MXID, IP Address or USB port name
     */
    explicit DeviceInfo(std::string mxidOrName);
    explicit DeviceInfo(const deviceDesc_t& desc);
    deviceDesc_t getXLinkDeviceDesc() const;
    std::string getMxId() const;
    std::string toString() const;

    std::string name = "";
    std::string mxid = "";
    XLinkDeviceState_t state = X_LINK_ANY_STATE;
    XLinkProtocol_t protocol = X_LINK_ANY_PROTOCOL;
    XLinkPlatform_t platform = X_LINK_ANY_PLATFORM;
    XLinkError_t status = X_LINK_SUCCESS;
};

/**
 * Represents connection between host and device over XLink protocol
 */
class XLinkConnection {
   public:
    // static API

    /**
     * Returns information of all connected devices with given state
     *
     * @param state State which the devices should be in
     * @param skipInvalidDevices whether or not to skip over devices that cannot be successfully communicated with
     * @returns Vector of connected device information
     */
    static std::vector<DeviceInfo> getAllConnectedDevices(XLinkDeviceState_t state = X_LINK_ANY_STATE, bool skipInvalidDevices = true);

    /**
     * Returns information of first device with given state
     * @param state State which the device should be in
     * @returns Device information
     */
    static std::tuple<bool, DeviceInfo> getFirstDevice(XLinkDeviceState_t state = X_LINK_ANY_STATE, bool skipInvalidDevices = true);

    /**
     * Finds a device by MX ID. Example: 14442C10D13EABCE00
     * @param mxId MyraidX ID which uniquely specifies a device
     * @param state Which state should the device be in
     * @param skipInvalidDevices Whether or not to skip devices that cannot be fully detected
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getDeviceByMxId(std::string mxId, XLinkDeviceState_t state = X_LINK_ANY_STATE, bool skipInvalidDevice = true);

    /**
     * Tries booting the given device into bootloader state
     *
     * @param devInfo Information of device which it should boot into bootloader state
     * @returns New device information if successful
     */
    static DeviceInfo bootBootloader(const DeviceInfo& devInfo);

    /**
     * Get current accumulated profiling data
     *
     * @returns ProfilingData from the specific connection
     */
    static ProfilingData getGlobalProfilingData();

    XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    XLinkConnection(const DeviceInfo& deviceDesc, dai::Path pathToMvcmd, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    explicit XLinkConnection(const DeviceInfo& deviceDesc, XLinkDeviceState_t expectedState = X_LINK_BOOTED);

    ~XLinkConnection();

    void setRebootOnDestruction(bool reboot);
    bool getRebootOnDestruction() const;

    int getLinkId() const;

    /**
     * Explicitly closes xlink connection.
     * @note This function does not need to be explicitly called
     * as destructor closes the connection automatically
     */
    void close();

    /**
     * Is the connection already closed (or disconnected)
     *
     * @warning This function is thread-unsafe and may return outdated incorrect values. It is
     * only meant for use in simple single-threaded code. Well written code should handle
     * exceptions when calling any DepthAI apis to handle hardware events and multithreaded use.
     */
    bool isClosed() const;

    /**
     * Get current accumulated profiling data
     *
     * @returns ProfilingData from the specific connection
     */
    ProfilingData getProfilingData();

   private:
    friend struct XLinkReadError;
    friend struct XLinkWriteError;
    // static
    static bool bootAvailableDevice(const deviceDesc_t& deviceToBoot, const dai::Path& pathToMvcmd);
    static bool bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd);
    static std::string convertErrorCodeToString(XLinkError_t errorCode);

    void initDevice(const DeviceInfo& deviceToInit, XLinkDeviceState_t expectedState = X_LINK_BOOTED);

    bool bootDevice = true;
    bool bootWithPath = true;
    dai::Path pathToMvcmd;
    std::vector<std::uint8_t> mvcmd;

    bool rebootOnDestruction{true};

    int deviceLinkId = -1;

    DeviceInfo deviceInfo;

    // closed
    mutable std::mutex closedMtx;
    bool closed{false};

    constexpr static std::chrono::milliseconds WAIT_FOR_BOOTUP_TIMEOUT{15000};
    constexpr static std::chrono::milliseconds WAIT_FOR_CONNECT_TIMEOUT{5000};
    constexpr static std::chrono::milliseconds POLLING_DELAY_TIME{10};
};

}  // namespace dai
