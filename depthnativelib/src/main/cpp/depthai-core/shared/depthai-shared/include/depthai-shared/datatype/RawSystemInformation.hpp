#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/common/ChipTemperature.hpp"
#include "depthai-shared/common/CpuUsage.hpp"
#include "depthai-shared/common/MemoryInfo.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {
/**
 * System information of device
 *
 * Memory usage, cpu usage and chip temperature
 */
struct RawSystemInformation : public RawBuffer {
    /// DDR memory usage
    MemoryInfo ddrMemoryUsage;
    /// CMX memory usage
    MemoryInfo cmxMemoryUsage;
    /// LeonCss heap usage
    MemoryInfo leonCssMemoryUsage;
    /// LeonMss heap usage
    MemoryInfo leonMssMemoryUsage;
    /// LeonCss cpu usage
    CpuUsage leonCssCpuUsage;
    /// LeonMss cpu usage
    CpuUsage leonMssCpuUsage;
    /// Chip temperatures
    ChipTemperature chipTemperature;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::SystemInformation;
    };

    DEPTHAI_SERIALIZE(
        RawSystemInformation, ddrMemoryUsage, cmxMemoryUsage, leonCssMemoryUsage, leonMssMemoryUsage, leonCssCpuUsage, leonMssCpuUsage, chipTemperature);
};

}  // namespace dai
