#pragma once

// std
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

// project
#include "DataQueue.hpp"
#include "depthai/device/DeviceBase.hpp"

namespace dai {
/**
 * Represents the DepthAI device with the methods to interact with it.
 * Implements the host-side queues to connect with XLinkIn and XLinkOut nodes
 */
class Device : public DeviceBase {
   public:
    using DeviceBase::DeviceBase;  // inherit the ctors

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     */
    explicit Device(const Pipeline& pipeline);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param usb2Mode (bool) Boot device using USB2 mode firmware
     */
    template <typename T, std::enable_if_t<std::is_same<T, bool>::value, bool> = true>
    [[deprecated("Use constructor taking 'UsbSpeed' instead")]] Device(const Pipeline& pipeline, T usb2Mode);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    Device(const Pipeline& pipeline, UsbSpeed maxUsbSpeed);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param pathToCmd Path to custom device firmware
     */
    Device(const Pipeline& pipeline, const dai::Path& pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     */
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode (bool) Boot device using USB2 mode firmware
     */
    template <typename T, std::enable_if_t<std::is_same<T, bool>::value, bool> = true>
    [[deprecated("Use constructor taking 'UsbSpeed' instead")]] Device(const Pipeline& pipeline, const DeviceInfo& devInfo, T usb2Mode);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, const dai::Path& pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version OpenVINO::VERSION_UNIVERSAL
     */
    Device();

    /**
     * @brief dtor to close the device
     */
    ~Device() override;

    /// Maximum number of elements in event queue
    static constexpr std::size_t EVENT_QUEUE_MAXIMUM_SIZE{2048};

    /**
     * Gets an output queue corresponding to stream name. If it doesn't exist it throws
     *
     * @param name Queue/stream name, created by XLinkOut node
     * @returns Smart pointer to DataOutputQueue
     */
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name);

    /**
     * Gets a queue corresponding to stream name, if it exists, otherwise it throws. Also sets queue options
     *
     * @param name Queue/stream name, set in XLinkOut node
     * @param maxSize Maximum number of messages in queue
     * @param blocking Queue behavior once full. True specifies blocking and false overwriting of oldest messages. Default: true
     * @returns Smart pointer to DataOutputQueue
     */
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name, unsigned int maxSize, bool blocking = true);

    /**
     * Get all available output queue names
     *
     * @returns Vector of output queue names
     */
    std::vector<std::string> getOutputQueueNames() const;

    /**
     * Gets an input queue corresponding to stream name. If it doesn't exist it throws
     *
     * @param name Queue/stream name, set in XLinkIn node
     * @returns Smart pointer to DataInputQueue
     */
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name);

    /**
     * Gets an input queue corresponding to stream name. If it doesn't exist it throws. Also sets queue options
     *
     * @param name Queue/stream name, set in XLinkOut node
     * @param maxSize Maximum number of messages in queue
     * @param blocking Queue behavior once full. True: blocking, false: overwriting of oldest messages. Default: true
     * @returns Smart pointer to DataInputQueue
     */
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name, unsigned int maxSize, bool blocking = true);

    /**
     * Get all available input queue names
     *
     * @returns Vector of input queue names
     */
    std::vector<std::string> getInputQueueNames() const;

    // void setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);

    /**
     * Gets or waits until any of specified queues has received a message
     *
     * @param queueNames Names of queues for which to block
     * @param maxNumEvents Maximum number of events to remove from queue - Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite - Default is -1
     * @returns Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(const std::vector<std::string>& queueNames,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));
    std::vector<std::string> getQueueEvents(const std::initializer_list<std::string>& queueNames,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until specified queue has received a message
     *
     * @param queueName Name of queues for which to wait for
     * @param maxNumEvents Maximum number of events to remove from queue. Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(std::string queueName,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any queue has received a message
     *
     * @param maxNumEvents Maximum number of events to remove from queue. Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any of specified queues has received a message
     *
     * @param queueNames Names of queues for which to wait for
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Queue name which received a message first
     */
    std::string getQueueEvent(const std::vector<std::string>& queueNames, std::chrono::microseconds timeout = std::chrono::microseconds(-1));
    std::string getQueueEvent(const std::initializer_list<std::string>& queueNames, std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until specified queue has received a message
     *
     * @param queueNames Name of queues for which to wait for
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Queue name which received a message
     */
    std::string getQueueEvent(std::string queueName, std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any queue has received a message
     *
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Queue name which received a message
     */
    std::string getQueueEvent(std::chrono::microseconds timeout = std::chrono::microseconds(-1));

   private:
    std::unordered_map<std::string, std::shared_ptr<DataOutputQueue>> outputQueueMap;
    std::unordered_map<std::string, std::shared_ptr<DataInputQueue>> inputQueueMap;
    std::unordered_map<std::string, DataOutputQueue::CallbackId> callbackIdMap;

    // Event queue
    std::mutex eventMtx;
    std::condition_variable eventCv;
    std::deque<std::string> eventQueue;

    bool startPipelineImpl(const Pipeline& pipeline) override;
    void closeImpl() override;
};

}  // namespace dai
