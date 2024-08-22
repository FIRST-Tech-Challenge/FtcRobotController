#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSystemInformation.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * SystemInformation message. Carries memory usage, cpu usage and chip temperatures.
 */
class SystemInformation : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSystemInformation& systemInformation;

   public:
    /**
     * Construct SystemInformation message.
     */
    SystemInformation();
    explicit SystemInformation(std::shared_ptr<RawSystemInformation> ptr);
    virtual ~SystemInformation() = default;

    MemoryInfo& ddrMemoryUsage;
    MemoryInfo& cmxMemoryUsage;
    MemoryInfo& leonCssMemoryUsage;
    MemoryInfo& leonMssMemoryUsage;
    CpuUsage& leonCssCpuUsage;
    CpuUsage& leonMssCpuUsage;
    ChipTemperature& chipTemperature;
};

}  // namespace dai
