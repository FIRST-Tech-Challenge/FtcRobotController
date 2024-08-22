#include "depthai-shared/utility/Checksum.hpp"

namespace dai {
namespace utility {

std::uint32_t checksum(const void* buffer, std::size_t size, uint32_t prevChecksum) {
    uint32_t checksum = prevChecksum;
    auto p = reinterpret_cast<const uint8_t*>(buffer);

    for(std::size_t i = 0; i < size; i++) {
        checksum = ((checksum << 5) + checksum) + p[i]; /* hash * 33 + p[i] */
    }

    return checksum;
}

std::uint32_t checksum(const void* buffer, std::size_t size) {
    constexpr static auto checksumInitialValue = 5381U;
    return checksum(buffer, size, checksumInitialValue);
}

}  // namespace utility
}  // namespace dai
