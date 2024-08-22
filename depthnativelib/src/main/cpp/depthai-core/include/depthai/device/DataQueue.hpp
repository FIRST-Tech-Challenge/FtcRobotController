#pragma once

// std
#include <atomic>
#include <memory>
#include <vector>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// shared
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

namespace dai {

/**
 * Access to receive messages coming from XLink stream
 */
class DataOutputQueue {
   public:
    /// Alias for callback id
    using CallbackId = int;

   private:
    LockingQueue<std::shared_ptr<ADatatype>> queue;
    std::thread readingThread;
    std::atomic<bool> running{true};
    std::string exceptionMessage{""};
    const std::string name{""};
    std::mutex callbacksMtx;
    std::unordered_map<CallbackId, std::function<void(std::string, std::shared_ptr<ADatatype>)>> callbacks;
    CallbackId uniqueCallbackId{0};

    // const std::chrono::milliseconds READ_TIMEOUT{500};

   public:
    // DataOutputQueue constructor
    DataOutputQueue(const std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize = 16, bool blocking = true);
    ~DataOutputQueue();

    /**
     * Check whether queue is closed
     *
     * @warning This function is thread-unsafe and may return outdated incorrect values. It is
     * only meant for use in simple single-threaded code. Well written code should handle
     * exceptions when calling any DepthAI apis to handle hardware events and multithreaded use.
     */
    bool isClosed() const;

    /**
     * Closes the queue and the underlying thread
     */
    void close();

    /**
     * Sets queue behavior when full (maxSize)
     *
     * @param blocking Specifies if block or overwrite the oldest message in the queue
     */
    void setBlocking(bool blocking);

    /**
     * Gets current queue behavior when full (maxSize)
     *
     * @returns True if blocking, false otherwise
     */
    bool getBlocking() const;

    /**
     * Sets queue maximum size
     *
     * @param maxSize Specifies maximum number of messages in the queue
     */
    void setMaxSize(unsigned int maxSize);

    /**
     * Gets queue maximum size
     *
     * @returns Maximum queue size
     */
    unsigned int getMaxSize() const;

    /**
     * Gets queues name
     *
     * @returns Queue name
     */
    std::string getName() const;

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function with queue name and message pointer
     * @returns Callback id
     */
    CallbackId addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)>);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function with message pointer
     * @returns Callback id
     */
    CallbackId addCallback(std::function<void(std::shared_ptr<ADatatype>)>);

    /**
     * Adds a callback on message received
     *
     * @param callback Callback function without any parameters
     * @returns Callback id
     */
    CallbackId addCallback(std::function<void()> callback);

    /**
     * Removes a callback
     *
     * @param callbackId Id of callback to be removed
     * @returns True if callback was removed, false otherwise
     */
    bool removeCallback(CallbackId callbackId);

    /**
     * Check whether front of the queue has message of type T
     * @returns True if queue isn't empty and the first element is of type T, false otherwise
     */
    template <class T>
    bool has() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(queue.front(val) && dynamic_cast<T*>(val.get())) {
            return true;
        }
        return false;
    }

    /**
     * Check whether front of the queue has a message (isn't empty)
     * @returns True if queue isn't empty, false otherwise
     */
    bool has() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        return !queue.empty();
    }

    /**
     * Try to retrieve message T from queue. If message isn't of type T it returns nullptr
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> tryGet() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryPop(val)) return nullptr;
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Try to retrieve message from queue. If no message available, return immediately with nullptr
     *
     * @returns Message or nullptr if no message available
     */
    std::shared_ptr<ADatatype> tryGet() {
        return tryGet<ADatatype>();
    }

    /**
     * Block until a message is available.
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> get() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.waitAndPop(val)) {
            throw std::runtime_error(exceptionMessage.c_str());
        }
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Block until a message is available.
     *
     * @returns Message or nullptr if no message available
     */
    std::shared_ptr<ADatatype> get() {
        return get<ADatatype>();
    }

    /**
     * Gets first message in the queue.
     *
     * @returns Message of type T or nullptr if no message available
     */
    template <class T>
    std::shared_ptr<T> front() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.front(val)) return nullptr;
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Gets first message in the queue.
     *
     * @returns Message or nullptr if no message available
     */
    std::shared_ptr<ADatatype> front() {
        return front<ADatatype>();
    }

    /**
     * Block until a message is available with a timeout.
     *
     * @param timeout Duration for which the function should block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Message of type T otherwise nullptr if message isn't type T or timeout occurred
     */
    template <class T, typename Rep, typename Period>
    std::shared_ptr<T> get(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());
        std::shared_ptr<ADatatype> val = nullptr;
        if(!queue.tryWaitAndPop(val, timeout)) {
            hasTimedout = true;
            return nullptr;
        }
        hasTimedout = false;
        return std::dynamic_pointer_cast<T>(val);
    }

    /**
     * Block until a message is available with a timeout.
     *
     * @param timeout Duration for which the function should block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Message of type T otherwise nullptr if message isn't type T or timeout occurred
     */
    template <typename Rep, typename Period>
    std::shared_ptr<ADatatype> get(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        return get<ADatatype>(timeout, hasTimedout);
    }

    /**
     * Try to retrieve all messages in the queue.
     *
     * @returns Vector of messages which can either be of type T or nullptr
     */
    template <class T>
    std::vector<std::shared_ptr<T>> tryGetAll() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());

        std::vector<std::shared_ptr<T>> messages;
        queue.consumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
            // dynamic pointer cast may return nullptr
            // in which case that message in vector will be nullptr
            messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
        });

        return messages;
    }

    /**
     * Try to retrieve all messages in the queue.
     *
     * @returns Vector of messages
     */
    std::vector<std::shared_ptr<ADatatype>> tryGetAll() {
        return tryGetAll<ADatatype>();
    }

    /**
     * Block until at least one message in the queue.
     * Then return all messages from the queue.
     *
     * @returns Vector of messages which can either be of type T or nullptr
     */
    template <class T>
    std::vector<std::shared_ptr<T>> getAll() {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());

        std::vector<std::shared_ptr<T>> messages;
        queue.waitAndConsumeAll([&messages](std::shared_ptr<ADatatype>& msg) {
            // dynamic pointer cast may return nullptr
            // in which case that message in vector will be nullptr
            messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
        });

        return messages;
    }

    /**
     * Block until at least one message in the queue.
     * Then return all messages from the queue.
     *
     * @returns Vector of messages
     */
    std::vector<std::shared_ptr<ADatatype>> getAll() {
        return getAll<ADatatype>();
    }

    /**
     * Block for maximum timeout duration.
     * Then return all messages from the queue.
     * @param timeout Maximum duration to block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Vector of messages which can either be of type T or nullptr
     */
    template <class T, typename Rep, typename Period>
    std::vector<std::shared_ptr<T>> getAll(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        if(!running) throw std::runtime_error(exceptionMessage.c_str());

        std::vector<std::shared_ptr<T>> messages;
        hasTimedout = !queue.waitAndConsumeAll(
            [&messages](std::shared_ptr<ADatatype>& msg) {
                // dynamic pointer cast may return nullptr
                // in which case that message in vector will be nullptr
                messages.push_back(std::dynamic_pointer_cast<T>(std::move(msg)));
            },
            timeout);

        return messages;
    }

    /**
     * Block for maximum timeout duration.
     * Then return all messages from the queue.
     * @param timeout Maximum duration to block
     * @param[out] hasTimedout Outputs true if timeout occurred, false otherwise
     * @returns Vector of messages
     */
    template <typename Rep, typename Period>
    std::vector<std::shared_ptr<ADatatype>> getAll(std::chrono::duration<Rep, Period> timeout, bool& hasTimedout) {
        return getAll<ADatatype>(timeout, hasTimedout);
    }
};

/**
 * Access to send messages through XLink stream
 */
class DataInputQueue {
    LockingQueue<std::shared_ptr<RawBuffer>> queue;
    std::thread writingThread;
    std::atomic<bool> running{true};
    std::string exceptionMessage;
    const std::string name;
    std::atomic<std::size_t> maxDataSize{device::XLINK_USB_BUFFER_MAX_SIZE};

   public:
    DataInputQueue(const std::shared_ptr<XLinkConnection> conn,
                   const std::string& streamName,
                   unsigned int maxSize = 16,
                   bool blocking = true,
                   std::size_t maxDataSize = device::XLINK_USB_BUFFER_MAX_SIZE);
    ~DataInputQueue();

    /**
     * Check whether queue is closed
     *
     * @warning This function is thread-unsafe and may return outdated incorrect values. It is
     * only meant for use in simple single-threaded code. Well written code should handle
     * exceptions when calling any DepthAI apis to handle hardware events and multithreaded use.
     */
    bool isClosed() const;

    /**
     * Closes the queue and the underlying thread
     */
    void close();

    /**
     * Sets maximum message size. If message is larger than specified, then an exception is issued.
     *
     * @param maxSize Maximum message size to add to queue
     */
    void setMaxDataSize(std::size_t maxSize);

    /**
     * Gets maximum queue size.
     *
     * @returns Maximum message size
     */
    std::size_t getMaxDataSize();

    /**
     * Sets queue behavior when full (maxSize)
     *
     * @param blocking Specifies if block or overwrite the oldest message in the queue
     */
    void setBlocking(bool blocking);

    /**
     * Gets current queue behavior when full (maxSize)
     *
     * @returns True if blocking, false otherwise
     */
    bool getBlocking() const;

    /**
     * Sets queue maximum size
     *
     * @param maxSize Specifies maximum number of messages in the queue
     */
    void setMaxSize(unsigned int maxSize);

    /**
     * Gets queue maximum size
     *
     * @returns Maximum queue size
     */
    unsigned int getMaxSize() const;

    /**
     * Gets queues name
     *
     * @returns Queue name
     */
    std::string getName() const;

    /**
     * Adds a raw message to the queue, which will be picked up and sent to the device.
     * Can either block if 'blocking' behavior is true or overwrite oldest
     * @param rawMsg Message to add to the queue
     */
    void send(const std::shared_ptr<RawBuffer>& rawMsg);

    /**
     * Adds a message to the queue, which will be picked up and sent to the device.
     * Can either block if 'blocking' behavior is true or overwrite oldest
     * @param msg Message to add to the queue
     */
    void send(const std::shared_ptr<ADatatype>& msg);

    /**
     * Adds a message to the queue, which will be picked up and sent to the device.
     * Can either block if 'blocking' behavior is true or overwrite oldest
     * @param msg Message to add to the queue
     */
    void send(const ADatatype& msg);

    /**
     * Adds message to the queue, which will be picked up and sent to the device.
     * Can either block until timeout if 'blocking' behavior is true or overwrite oldest
     *
     * @param rawMsg Message to add to the queue
     * @param timeout Maximum duration to block in milliseconds
     */
    bool send(const std::shared_ptr<RawBuffer>& rawMsg, std::chrono::milliseconds timeout);

    /**
     * Adds message to the queue, which will be picked up and sent to the device.
     * Can either block until timeout if 'blocking' behavior is true or overwrite oldest
     *
     * @param msg Message to add to the queue
     * @param timeout Maximum duration to block in milliseconds
     */
    bool send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout);

    /**
     * Adds message to the queue, which will be picked up and sent to the device.
     * Can either block until timeout if 'blocking' behavior is true or overwrite oldest
     *
     * @param msg Message to add to the queue
     * @param timeout Maximum duration to block in milliseconds
     */
    bool send(const ADatatype& msg, std::chrono::milliseconds timeout);
};

}  // namespace dai
