#pragma once

#include <depthai-shared/common/Timestamp.hpp>

namespace dai {

struct TraceEvent {
    enum Event : std::uint8_t {
        SEND,
        RECEIVE,
        // PULL,
    };
    enum class Status : std::uint8_t {
        START,
        END,
        TIMEOUT,
    };
    Event event;
    Status status;
    uint32_t srcId;
    uint32_t dstId;
    Timestamp timestamp;
};

}  // namespace dai
