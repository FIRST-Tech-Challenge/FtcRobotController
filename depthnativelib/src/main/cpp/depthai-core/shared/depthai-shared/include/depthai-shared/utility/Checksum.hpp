#pragma once

// std
#include <cstddef>
#include <cstdint>

namespace dai {
namespace utility {

/**
 * Simple hash function - djb2
 * @param buffer Pointer to buffer of data to hash
 * @param size Size of buffer in bytes
 * @param prevChecksum Previous checksum - useful for doing hash on blocks of data
 */
std::uint32_t checksum(const void* buffer, std::size_t size, uint32_t prevChecksum);

/**
 * Simple hash function - djb2
 * @param buffer Pointer to buffer of data to hash
 * @param size Size of buffer in bytes
 */
std::uint32_t checksum(const void* buffer, std::size_t size);

}  // namespace utility
}  // namespace dai
