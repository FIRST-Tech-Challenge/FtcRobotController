#pragma once

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * MemoryInfo structure
 *
 * Free, remaining and total memory stats
 */
struct MemoryInfo {
    int64_t remaining;
    int64_t used;
    int64_t total;
};

DEPTHAI_SERIALIZE_EXT(MemoryInfo, remaining, used, total);

}  // namespace dai