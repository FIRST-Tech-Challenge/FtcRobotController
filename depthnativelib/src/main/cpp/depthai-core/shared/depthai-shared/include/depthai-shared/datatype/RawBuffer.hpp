#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/common/Timestamp.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawBuffer structure
struct RawBuffer {
    virtual ~RawBuffer() = default;
    std::vector<std::uint8_t> data;

    int64_t sequenceNum = 0;
    Timestamp ts = {};
    Timestamp tsDevice = {};

    virtual void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Buffer;
    };
    DEPTHAI_SERIALIZE(RawBuffer, sequenceNum, ts, tsDevice);
};

}  // namespace dai
