#pragma once

// Std
#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// libraries
#include <XLink/XLinkPublicDefines.h>
#include <XLink/XLinkTime.h>

// project
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

class StreamPacketDesc : public streamPacketDesc_t {
   public:
    StreamPacketDesc() noexcept : streamPacketDesc_t{nullptr, 0, {}, {}} {};
    StreamPacketDesc(const StreamPacketDesc&) = delete;
    StreamPacketDesc(StreamPacketDesc&& other) noexcept;
    StreamPacketDesc& operator=(const StreamPacketDesc&) = delete;
    StreamPacketDesc& operator=(StreamPacketDesc&& other) noexcept;
    ~StreamPacketDesc() noexcept;
};

class XLinkStream {
    // static
    constexpr static int STREAM_OPEN_RETRIES = 5;
    constexpr static std::chrono::milliseconds WAIT_FOR_STREAM_RETRY{50};

    std::shared_ptr<XLinkConnection> connection;
    std::string streamName;
    streamId_t streamId{INVALID_STREAM_ID};

   public:
    XLinkStream(const std::shared_ptr<XLinkConnection> conn, const std::string& name, std::size_t maxWriteSize);
    XLinkStream(const XLinkStream&) = delete;
    XLinkStream(XLinkStream&& stream);
    XLinkStream& operator=(const XLinkStream&) = delete;
    XLinkStream& operator=(XLinkStream&& stream);
    ~XLinkStream();

    // Blocking
    void write(const void* data, std::size_t size);
    void write(const std::uint8_t* data, std::size_t size);
    void write(const std::vector<std::uint8_t>& data);
    std::vector<std::uint8_t> read();
    std::vector<std::uint8_t> read(XLinkTimespec& timestampReceived);
    void read(std::vector<std::uint8_t>& data);
    void read(std::vector<std::uint8_t>& data, XLinkTimespec& timestampReceived);
    // split write helper
    void writeSplit(const void* data, std::size_t size, std::size_t split);
    void writeSplit(const std::vector<uint8_t>& data, std::size_t split);
    StreamPacketDesc readMove();

    // Timeout
    bool write(const void* data, std::size_t size, std::chrono::milliseconds timeout);
    bool write(const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout);
    bool write(const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    bool read(std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    bool readMove(StreamPacketDesc& packet, const std::chrono::milliseconds timeout);
    // TODO optional<StreamPacketDesc> readMove(timeout) -or- tuple<bool, StreamPacketDesc> readMove(timeout)

    // deprecated use readMove() instead; readRaw leads to memory violations and/or memory leaks
    [[deprecated("use readMove()")]] streamPacketDesc_t* readRaw();
    // deprecated use readMove(packet, timeout) instead; readRaw leads to memory violations and/or memory leaks
    [[deprecated("use readMove(packet, timeout)")]] bool readRaw(streamPacketDesc_t*& pPacket, std::chrono::milliseconds timeout);
    // deprecated; unsafe leads to memory violations and/or memory leaks
    [[deprecated]] void readRawRelease();

    streamId_t getStreamId() const;
};

struct XLinkError : public std::runtime_error {
    const XLinkError_t status = X_LINK_ERROR;
    const std::string streamName;

    using std::runtime_error::runtime_error;

    XLinkError(XLinkError_t statusID, std::string stream, const std::string& message)
        : runtime_error(message), status(statusID), streamName(std::move(stream)) {}
};
struct XLinkReadError : public XLinkError {
    using XLinkError = XLinkError;
    XLinkReadError(XLinkError_t status, const std::string& stream);
};
struct XLinkWriteError : public XLinkError {
    using XLinkError = XLinkError;
    XLinkWriteError(XLinkError_t status, const std::string& stream);
};

}  // namespace dai
