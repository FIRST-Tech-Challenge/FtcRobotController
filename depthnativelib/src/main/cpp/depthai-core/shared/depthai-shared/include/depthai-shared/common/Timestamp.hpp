#pragma once

// std
#include <chrono>
#include <cstdint>

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// Timestamp structure
struct Timestamp {
    int64_t sec = 0, nsec = 0;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> get() const {
        using namespace std::chrono;
        return time_point<steady_clock, steady_clock::duration>{seconds(sec) + nanoseconds(nsec)};
    }
};

DEPTHAI_SERIALIZE_EXT(Timestamp, sec, nsec);

}  // namespace dai
