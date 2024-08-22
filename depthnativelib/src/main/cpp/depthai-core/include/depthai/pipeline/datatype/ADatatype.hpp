#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

/// Abstract message
class ADatatype {
   protected:
    friend class DataInputQueue;
    friend class StreamMessageParser;
    std::shared_ptr<RawBuffer> raw;

   public:
    explicit ADatatype(std::shared_ptr<RawBuffer> r) : raw(std::move(r)) {}
    virtual ~ADatatype() = default;
    virtual std::shared_ptr<dai::RawBuffer> serialize() const = 0;
    std::shared_ptr<RawBuffer> getRaw() const {
        return raw;
    }
};

}  // namespace dai
