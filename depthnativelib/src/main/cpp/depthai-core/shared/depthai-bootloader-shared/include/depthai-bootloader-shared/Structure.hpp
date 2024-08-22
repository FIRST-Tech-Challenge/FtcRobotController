#pragma once

// std
#include <map>
#include <stdexcept>

// project
#include "Section.hpp"
#include "Type.hpp"

namespace dai
{
namespace bootloader
{

// Memory section structure
struct Structure {
    Structure() = default;
    std::map<Section, long> offset, size;
protected:
    Structure(decltype(offset) a, decltype(size) b) : offset(a), size(b) {}
};

// Structure
struct NetworkBootloaderStructure : Structure {

    constexpr static long HEADER_OFFSET = 0;
    constexpr static long HEADER_SIZE = 512;
    constexpr static long CONFIG_SIZE = 16 * 1024;
    constexpr static long BOOTLOADER_OFFSET = HEADER_OFFSET + HEADER_SIZE;
    constexpr static long BOOTLOADER_SIZE = 4 * 1024 * 1024 - HEADER_SIZE;
    constexpr static long USER_BOOTLOADER_OFFSET = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
    constexpr static long USER_BOOTLOADER_SIZE = 8 * 1024 * 1024 - BOOTLOADER_SIZE - CONFIG_SIZE - HEADER_SIZE;
    constexpr static long CONFIG_OFFSET = USER_BOOTLOADER_OFFSET + USER_BOOTLOADER_SIZE;
    constexpr static long APPLICATION_OFFSET = CONFIG_OFFSET + CONFIG_SIZE;

    NetworkBootloaderStructure() : Structure({
        {Section::HEADER, HEADER_OFFSET},
        {Section::BOOTLOADER_CONFIG, CONFIG_OFFSET},
        {Section::BOOTLOADER, BOOTLOADER_OFFSET},
        {Section::USER_BOOTLOADER, USER_BOOTLOADER_OFFSET},
        {Section::APPLICATION, APPLICATION_OFFSET},
    }, {
        {Section::HEADER, HEADER_SIZE},
        {Section::BOOTLOADER_CONFIG, CONFIG_SIZE},
        {Section::BOOTLOADER, BOOTLOADER_SIZE},
        {Section::USER_BOOTLOADER, USER_BOOTLOADER_SIZE},
        {Section::APPLICATION, 0},
    }) {}

};

// Structure
struct UsbBootloaderStructure : Structure {

    constexpr static long HEADER_OFFSET = 0;
    constexpr static long HEADER_SIZE = 512;
    constexpr static long CONFIG_SIZE = 16 * 1024;
    constexpr static long BOOTLOADER_OFFSET = HEADER_OFFSET + HEADER_SIZE;
    constexpr static long BOOTLOADER_SIZE = 1 * 1024 * 1024 - CONFIG_SIZE - HEADER_SIZE;
    constexpr static long CONFIG_OFFSET = BOOTLOADER_OFFSET + BOOTLOADER_SIZE;
    constexpr static long APPLICATION_OFFSET = CONFIG_OFFSET + CONFIG_SIZE;

    UsbBootloaderStructure() : Structure({
        {Section::HEADER, HEADER_OFFSET},
        {Section::BOOTLOADER_CONFIG, CONFIG_OFFSET},
        {Section::BOOTLOADER, BOOTLOADER_OFFSET},
        {Section::APPLICATION, APPLICATION_OFFSET},
    }, {
        {Section::HEADER, HEADER_SIZE},
        {Section::BOOTLOADER_CONFIG, CONFIG_SIZE},
        {Section::BOOTLOADER, BOOTLOADER_SIZE},
        {Section::APPLICATION, 0},
    }) {}

};

inline const Structure getStructure(Type type){
    switch(type){
        case Type::AUTO: throw std::invalid_argument("Invalid argument to getStructure function");
        case Type::USB: return UsbBootloaderStructure();
        case Type::NETWORK: return NetworkBootloaderStructure();
    }
    // Default
    return UsbBootloaderStructure();
}

} // namespace bootloader
} // namespace dai

