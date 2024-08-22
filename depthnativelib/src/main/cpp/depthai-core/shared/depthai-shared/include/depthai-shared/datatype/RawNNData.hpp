#pragma once

#include "depthai-shared/common/TensorInfo.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawNNData structure
struct RawNNData : public RawBuffer {
    // NNData data is in PoBuf
    std::vector<TensorInfo> tensors;
    unsigned int batchSize;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::NNData;
    };

    DEPTHAI_SERIALIZE(RawNNData, tensors, batchSize, RawBuffer::sequenceNum, RawBuffer::ts, RawBuffer::tsDevice);
};

}  // namespace dai
