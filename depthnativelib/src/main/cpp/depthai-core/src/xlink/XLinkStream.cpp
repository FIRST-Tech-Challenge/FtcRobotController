#include "depthai/xlink/XLinkStream.hpp"

// libraries
#include "XLink/XLink.h"
#include "spdlog/fmt/fmt.h"

// project
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

// static
constexpr std::chrono::milliseconds XLinkStream::WAIT_FOR_STREAM_RETRY;
constexpr int XLinkStream::STREAM_OPEN_RETRIES;

XLinkStream::XLinkStream(const std::shared_ptr<XLinkConnection> conn, const std::string& name, std::size_t maxWriteSize) : connection(conn), streamName(name) {
    if(name.empty()) throw std::invalid_argument("Cannot create XLinkStream using empty stream name");
    if(!connection || connection->getLinkId() == -1) throw std::invalid_argument("Cannot create XLinkStream using unconnected XLinkConnection");

    streamId = INVALID_STREAM_ID;

    for(int retryCount = 0; retryCount < STREAM_OPEN_RETRIES; retryCount++) {
        streamId = XLinkOpenStream(connection->getLinkId(), streamName.c_str(), static_cast<int>(maxWriteSize));
        if(streamId == INVALID_STREAM_ID) {
            // Give some time before continuing
            std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);
        } else {
            break;
        }
    }

    if(streamId == INVALID_STREAM_ID) throw std::runtime_error("Couldn't open stream");
}

// Move constructor
XLinkStream::XLinkStream(XLinkStream&& other)
    : connection(std::move(other.connection)), streamName(std::exchange(other.streamName, {})), streamId(std::exchange(other.streamId, INVALID_STREAM_ID)) {
    // Set other's streamId to INVALID_STREAM_ID to prevent closing
}

XLinkStream& XLinkStream::operator=(XLinkStream&& other) {
    if(this != &other) {
        connection = std::move(other.connection);
        streamId = std::exchange(other.streamId, INVALID_STREAM_ID);
        streamName = std::exchange(other.streamName, {});
    }
    return *this;
}

XLinkStream::~XLinkStream() {
    // If streamId != invalid (eg. wasn't moved to another XLinkStream)
    if(streamId != INVALID_STREAM_ID) {
        XLinkCloseStream(streamId);
    }
}

StreamPacketDesc::StreamPacketDesc(StreamPacketDesc&& other) noexcept : streamPacketDesc_t{other.data, other.length, other.tRemoteSent, other.tReceived} {
    other.data = nullptr;
    other.length = 0;
}

StreamPacketDesc& StreamPacketDesc::operator=(StreamPacketDesc&& other) noexcept {
    if(this != &other) {
        data = std::exchange(other.data, nullptr);
        length = std::exchange(other.length, 0);
        tRemoteSent = std::exchange(other.tRemoteSent, {});
        tReceived = std::exchange(other.tReceived, {});
    }
    return *this;
}

StreamPacketDesc::~StreamPacketDesc() noexcept {
    XLinkDeallocateMoveData(data, length);
}

////////////////////
// BLOCKING VERSIONS
////////////////////

void XLinkStream::write(const std::uint8_t* data, std::size_t size) {
    auto status = XLinkWriteData(streamId, data, static_cast<int>(size));
    if(status != X_LINK_SUCCESS) {
        throw XLinkWriteError(status, streamName);
    }
}
void XLinkStream::write(const void* data, std::size_t size) {
    write(reinterpret_cast<const uint8_t*>(data), size);
}

void XLinkStream::write(const std::vector<std::uint8_t>& data) {
    write(data.data(), data.size());
}

void XLinkStream::read(std::vector<std::uint8_t>& data) {
    StreamPacketDesc packet;
    const auto status = XLinkReadMoveData(streamId, &packet);
    if(status != X_LINK_SUCCESS) {
        throw XLinkReadError(status, streamName);
    }
    data = std::vector<std::uint8_t>(packet.data, packet.data + packet.length);
}

void XLinkStream::read(std::vector<std::uint8_t>& data, XLinkTimespec& timestampReceived) {
    StreamPacketDesc packet;
    const auto status = XLinkReadMoveData(streamId, &packet);
    if(status != X_LINK_SUCCESS) {
        throw XLinkReadError(status, streamName);
    }
    data = std::vector<std::uint8_t>(packet.data, packet.data + packet.length);
    timestampReceived = packet.tReceived;
}

std::vector<std::uint8_t> XLinkStream::read() {
    std::vector<std::uint8_t> data;
    read(data);
    return data;
}

std::vector<std::uint8_t> XLinkStream::read(XLinkTimespec& timestampReceived) {
    std::vector<std::uint8_t> data;
    read(data, timestampReceived);
    return data;
}

StreamPacketDesc XLinkStream::readMove() {
    StreamPacketDesc packet;
    const auto status = XLinkReadMoveData(streamId, &packet);
    if(status != X_LINK_SUCCESS) {
        throw XLinkReadError(status, streamName);
    }
    return packet;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
streamPacketDesc_t* XLinkStream::readRaw() {
    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamId, &pPacket);
    if(status != X_LINK_SUCCESS) {
        throw XLinkReadError(status, streamName);
    }
    return pPacket;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
void XLinkStream::readRawRelease() {
    XLinkError_t status;
    if((status = XLinkReleaseData(streamId)) != X_LINK_SUCCESS) throw XLinkReadError(status, streamName);
}

// SPLIT HELPER
void XLinkStream::writeSplit(const void* d, std::size_t size, std::size_t split) {
    const uint8_t* data = (const uint8_t*)d;
    std::size_t currentOffset = 0;
    std::size_t remaining = size;
    std::size_t sizeToTransmit = 0;
    XLinkError_t ret = X_LINK_SUCCESS;
    while(remaining > 0) {
        sizeToTransmit = remaining > split ? split : remaining;
        ret = XLinkWriteData(streamId, data + currentOffset, static_cast<int>(sizeToTransmit));
        if(ret != X_LINK_SUCCESS) {
            throw XLinkWriteError(ret, streamName);
        }
        currentOffset += sizeToTransmit;
        remaining = size - currentOffset;
    }
}

void XLinkStream::writeSplit(const std::vector<uint8_t>& data, std::size_t split) {
    writeSplit(data.data(), data.size(), split);
}

///////////////////////
// Timeout versions //
//////////////////////

bool XLinkStream::write(const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout) {
    auto status = XLinkWriteDataWithTimeout(streamId, data, static_cast<int>(size), static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw XLinkWriteError(status, streamName);
    }
}

bool XLinkStream::write(const void* data, std::size_t size, std::chrono::milliseconds timeout) {
    return write(reinterpret_cast<const std::uint8_t*>(data), size, timeout);
}

bool XLinkStream::write(const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
    return write(data.data(), data.size(), timeout);
}

bool XLinkStream::read(std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
    StreamPacketDesc packet;
    const auto status = XLinkReadMoveDataWithTimeout(streamId, &packet, static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        data = std::vector<std::uint8_t>(packet.data, packet.data + packet.length);
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw XLinkReadError(status, streamName);
    }
}

bool XLinkStream::readMove(StreamPacketDesc& packet, const std::chrono::milliseconds timeout) {
    const auto status = XLinkReadMoveDataWithTimeout(streamId, &packet, static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw XLinkReadError(status, streamName);
    }
}

bool XLinkStream::readRaw(streamPacketDesc_t*& pPacket, std::chrono::milliseconds timeout) {
    auto status = XLinkReadDataWithTimeout(streamId, &pPacket, static_cast<unsigned int>(timeout.count()));
    if(status == X_LINK_SUCCESS) {
        return true;
    } else if(status == X_LINK_TIMEOUT) {
        return false;
    } else {
        throw XLinkReadError(status, streamName);
    }
}

streamId_t XLinkStream::getStreamId() const {
    return streamId;
}

XLinkReadError::XLinkReadError(XLinkError_t status, const std::string& streamName)
    : XLinkError(status, streamName, fmt::format("Couldn't read data from stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status))) {}

XLinkWriteError::XLinkWriteError(XLinkError_t status, const std::string& streamName)
    : XLinkError(status, streamName, fmt::format("Couldn't write data to stream: '{}' ({})", streamName, XLinkConnection::convertErrorCodeToString(status))) {}

}  // namespace dai
