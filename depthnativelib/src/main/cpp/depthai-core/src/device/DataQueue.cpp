#include "depthai/device/DataQueue.hpp"

// std
#include <chrono>
#include <iostream>

// project
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/xlink/XLinkStream.hpp"
#include "pipeline/datatype/StreamMessageParser.hpp"

// shared
#include "depthai-shared/xlink/XLinkConstants.hpp"

// libraries
#include "utility/Logging.hpp"

// Additions
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"

namespace dai {

// DATA OUTPUT QUEUE
DataOutputQueue::DataOutputQueue(const std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool blocking)
    : queue(maxSize, blocking), name(streamName) {
    // Create stream first and then pass to thread
    // Open stream with 1B write size (no writing will happen here)
    XLinkStream stream(std::move(conn), name, 1);

    // Creates a thread which reads from connection into the queue
    readingThread = std::thread([this, stream = std::move(stream)]() mutable {
        std::uint64_t numPacketsRead = 0;
        try {
            while(running) {
                // Blocking -- parse packet and gather timing information
                auto packet = stream.readMove();
                const auto t1Parse = std::chrono::steady_clock::now();
                const auto data = StreamMessageParser::parseMessageToADatatype(&packet);
                const auto t2Parse = std::chrono::steady_clock::now();

                // Trace level debugging
                if(logger::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    data->getRaw()->serialize(metadata, type);
                    logger::trace("Received message from device ({}) - parsing time: {}, data size: {}, object type: {} object data: {}",
                                  name,
                                  std::chrono::duration_cast<std::chrono::microseconds>(t2Parse - t1Parse),
                                  data->getRaw()->data.size(),
                                  static_cast<std::int32_t>(type),
                                  spdlog::to_hex(metadata));
                }

                // Add 'data' to queue
                if(!queue.push(data)) {
                    throw std::runtime_error(fmt::format("Underlying queue destructed"));
                }

                // Increment numPacketsRead
                numPacketsRead++;

                // Call callbacks
                {
                    std::unique_lock<std::mutex> l(callbacksMtx);
                    for(const auto& kv : callbacks) {
                        const auto& callback = kv.second;
                        try {
                            callback(name, data);
                        } catch(const std::exception& ex) {
                            logger::error("Callback with id: {} throwed an exception: {}", kv.first, ex.what());
                        }
                    }
                }
            }

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        // Close the queue
        close();
    });
}

// This function is thread-unsafe. The idea of "isClosed" is ephemerial and
// since there is no mutex lock, its state is outdated and invalid even before
// the logical NOT in this function. This calculated boolean then continues to degrade
// in validity as it is returned by value to the caller
bool DataOutputQueue::isClosed() const {
    return !running;
}

void DataOutputQueue::close() {
    // Set reading thread to stop and allow to be closed only once
    if(!running.exchange(false)) return;

    // Destroy queue
    queue.destruct();

    // Then join thread
    if((readingThread.get_id() != std::this_thread::get_id()) && readingThread.joinable()) readingThread.join();

    // Log
    logger::debug("DataOutputQueue ({}) closed", name);
}

DataOutputQueue::~DataOutputQueue() {
    // Close the queue first
    close();

    // Then join thread
    if(readingThread.joinable()) readingThread.join();
}

void DataOutputQueue::setBlocking(bool blocking) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setBlocking(blocking);
}

bool DataOutputQueue::getBlocking() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getBlocking();
}

void DataOutputQueue::setMaxSize(unsigned int maxSize) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setMaxSize(maxSize);
}

unsigned int DataOutputQueue::getMaxSize() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getMaxSize();
}

std::string DataOutputQueue::getName() const {
    return name;
}

int DataOutputQueue::addCallback(std::function<void(std::string, std::shared_ptr<ADatatype>)> callback) {
    // Lock first
    std::unique_lock<std::mutex> l(callbacksMtx);

    // Get unique id
    int id = uniqueCallbackId++;

    // move assign callback
    callbacks[id] = std::move(callback);

    // return id assigned to the callback
    return id;
}

int DataOutputQueue::addCallback(std::function<void(std::shared_ptr<ADatatype>)> callback) {
    // Create a wrapper
    return addCallback([callback = std::move(callback)](std::string, std::shared_ptr<ADatatype> message) { callback(std::move(message)); });
}

int DataOutputQueue::addCallback(std::function<void()> callback) {
    // Create a wrapper
    return addCallback([callback = std::move(callback)](std::string, std::shared_ptr<ADatatype>) { callback(); });
}

bool DataOutputQueue::removeCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> l(callbacksMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(callbacks.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    callbacks.erase(callbackId);
    return true;
}

// DATA INPUT QUEUE
DataInputQueue::DataInputQueue(
    const std::shared_ptr<XLinkConnection> conn, const std::string& streamName, unsigned int maxSize, bool blocking, std::size_t maxDataSize)
    : queue(maxSize, blocking), name(streamName), maxDataSize(maxDataSize) {
    // open stream with maxDataSize write size
    XLinkStream stream(std::move(conn), name, maxDataSize + device::XLINK_MESSAGE_METADATA_MAX_SIZE);

    writingThread = std::thread([this, stream = std::move(stream)]() mutable {
        std::uint64_t numPacketsSent = 0;
        try {
            while(running) {
                // get data from queue
                std::shared_ptr<RawBuffer> data;
                if(!queue.waitAndPop(data)) {
                    continue;
                }

                // serialize
                auto t1Parse = std::chrono::steady_clock::now();
                auto serialized = StreamMessageParser::serializeMessage(data);
                auto t2Parse = std::chrono::steady_clock::now();

                // Trace level debugging
                if(logger::get_level() == spdlog::level::trace) {
                    std::vector<std::uint8_t> metadata;
                    DatatypeEnum type;
                    data->serialize(metadata, type);
                    logger::trace("Sending message to device ({}) - serialize time: {}, data size: {}, object type: {} object data: {}",
                                  name,
                                  std::chrono::duration_cast<std::chrono::microseconds>(t2Parse - t1Parse),
                                  data->data.size(),
                                  type,
                                  spdlog::to_hex(metadata));
                }

                // Blocking
                stream.write(serialized);

                // Increment num packets sent
                numPacketsSent++;
            }

        } catch(const std::exception& ex) {
            exceptionMessage = fmt::format("Communication exception - possible device error/misconfiguration. Original message '{}'", ex.what());
        }

        // Close the queue
        close();
    });
}

// This function is thread-unsafe. The idea of "isClosed" is ephemerial and
// since there is no mutex lock, its state is outdated and invalid even before
// the logical NOT in this function. This calculated boolean then continues to degrade
// in validity as it is returned by value to the caller
bool DataInputQueue::isClosed() const {
    return !running;
}

void DataInputQueue::close() {
    // Set writing thread to stop and allow to be closed only once
    if(!running.exchange(false)) return;

    // Destroy queue
    queue.destruct();

    // Then join thread
    if((writingThread.get_id() != std::this_thread::get_id()) && writingThread.joinable()) writingThread.join();

    // Log
    logger::debug("DataInputQueue ({}) closed", name);
}

DataInputQueue::~DataInputQueue() {
    // Close the queue
    close();

    // Then join thread
    if(writingThread.joinable()) writingThread.join();
}

void DataInputQueue::setBlocking(bool blocking) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setBlocking(blocking);
}

bool DataInputQueue::getBlocking() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getBlocking();
}

void DataInputQueue::setMaxSize(unsigned int maxSize) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    queue.setMaxSize(maxSize);
}

unsigned int DataInputQueue::getMaxSize() const {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    return queue.getMaxSize();
}

// BUGBUG https://github.com/luxonis/depthai-core/issues/762
void DataInputQueue::setMaxDataSize(std::size_t maxSize) {
    maxDataSize = maxSize;
}

std::size_t DataInputQueue::getMaxDataSize() {
    return maxDataSize;
}

std::string DataInputQueue::getName() const {
    return name;
}

void DataInputQueue::send(const std::shared_ptr<RawBuffer>& rawMsg) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    if(!rawMsg) throw std::invalid_argument("Message passed is not valid (nullptr)");

    // Check if stream receiver has enough space for this message
    if(rawMsg->data.size() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", rawMsg->data.size(), maxDataSize));
    }

    if(!queue.push(rawMsg)) {
        throw std::runtime_error("Underlying queue destructed");
    }
}
void DataInputQueue::send(const std::shared_ptr<ADatatype>& msg) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    send(msg->serialize());
}

void DataInputQueue::send(const ADatatype& msg) {
    send(msg.serialize());
}

bool DataInputQueue::send(const std::shared_ptr<RawBuffer>& rawMsg, std::chrono::milliseconds timeout) {
    if(!running) throw std::runtime_error(exceptionMessage.c_str());
    if(!rawMsg) throw std::invalid_argument("Message passed is not valid (nullptr)");

    // Check if stream receiver has enough space for this message
    if(rawMsg->data.size() > maxDataSize) {
        throw std::runtime_error(fmt::format("Trying to send larger ({}B) message than XLinkIn maxDataSize ({}B)", rawMsg->data.size(), maxDataSize));
    }

    return queue.tryWaitAndPush(rawMsg, timeout);
}

bool DataInputQueue::send(const std::shared_ptr<ADatatype>& msg, std::chrono::milliseconds timeout) {
    if(!msg) throw std::invalid_argument("Message passed is not valid (nullptr)");
    return send(msg->serialize(), timeout);
}

bool DataInputQueue::send(const ADatatype& msg, std::chrono::milliseconds timeout) {
    return send(msg.serialize(), timeout);
}

}  // namespace dai
