#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"

namespace dai {

/// Base message - buffer of binary data
class Buffer : public ADatatype {
    std::shared_ptr<dai::RawBuffer> serialize() const override;

   public:
    /// Creates Buffer message
    Buffer();
    explicit Buffer(std::shared_ptr<dai::RawBuffer> ptr);
    virtual ~Buffer() = default;

    // helpers
    /**
     * @brief Get non-owning reference to internal buffer
     * @returns Reference to internal buffer
     */
    std::vector<std::uint8_t>& getData() const;

    /**
     * @param data Copies data to internal buffer
     */
    void setData(const std::vector<std::uint8_t>& data);

    /**
     * @param data Moves data to internal buffer
     */
    void setData(std::vector<std::uint8_t>&& data);

    /**
     * Retrieves timestamp related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp() const;

    /**
     * Retrieves timestamp directly captured from device's monotonic clock,
     * not synchronized to host time. Used mostly for debugging
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice() const;

    /**
     * Retrieves sequence number
     */
    int64_t getSequenceNum() const;

    /**
     * Sets timestamp related to dai::Clock::now()
     */
    Buffer& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets timestamp related to dai::Clock::now()
     */
    Buffer& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves sequence number
     */
    Buffer& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
