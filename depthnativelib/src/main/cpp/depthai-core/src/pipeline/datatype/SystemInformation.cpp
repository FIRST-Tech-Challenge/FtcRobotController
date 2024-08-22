#include "depthai/pipeline/datatype/SystemInformation.hpp"

namespace dai {

std::shared_ptr<RawBuffer> SystemInformation::serialize() const {
    return raw;
}

SystemInformation::SystemInformation()
    : Buffer(std::make_shared<RawSystemInformation>()),
      systemInformation(*dynamic_cast<RawSystemInformation*>(raw.get())),
      ddrMemoryUsage(systemInformation.ddrMemoryUsage),
      cmxMemoryUsage(systemInformation.cmxMemoryUsage),
      leonCssMemoryUsage(systemInformation.leonCssMemoryUsage),
      leonMssMemoryUsage(systemInformation.leonMssMemoryUsage),
      leonCssCpuUsage(systemInformation.leonCssCpuUsage),
      leonMssCpuUsage(systemInformation.leonMssCpuUsage),
      chipTemperature(systemInformation.chipTemperature) {}

SystemInformation::SystemInformation(std::shared_ptr<RawSystemInformation> ptr)
    : Buffer(std::move(ptr)),
      systemInformation(*dynamic_cast<RawSystemInformation*>(raw.get())),
      ddrMemoryUsage(systemInformation.ddrMemoryUsage),
      cmxMemoryUsage(systemInformation.cmxMemoryUsage),
      leonCssMemoryUsage(systemInformation.leonCssMemoryUsage),
      leonMssMemoryUsage(systemInformation.leonMssMemoryUsage),
      leonCssCpuUsage(systemInformation.leonCssCpuUsage),
      leonMssCpuUsage(systemInformation.leonMssCpuUsage),
      chipTemperature(systemInformation.chipTemperature) {}

}  // namespace dai
