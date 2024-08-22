#pragma once

#include "depthai-shared/common/EepromData.hpp"
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties which apply for whole pipeline
 */
struct GlobalProperties : PropertiesSerializable<Properties, GlobalProperties> {
    constexpr static uint32_t SIPP_BUFFER_DEFAULT_SIZE = 18 * 1024;
    constexpr static uint32_t SIPP_DMA_BUFFER_DEFAULT_SIZE = 16 * 1024;

    /**
     * Set frequency of Leon OS - Increasing can improve performance, at the cost of higher power
     * draw
     */
    double leonCssFrequencyHz = 700 * 1000 * 1000;
    /**
     * Set frequency of Leon RT - Increasing can improve performance, at the cost of higher power
     * draw
     */
    double leonMssFrequencyHz = 700 * 1000 * 1000;
    tl::optional<std::string> pipelineName;
    tl::optional<std::string> pipelineVersion;
    /**
     * Calibration data sent through pipeline
     */

    tl::optional<dai::EepromData> calibData;

    /**
     * Camera tuning blob size in bytes
     */
    tl::optional<std::uint32_t> cameraTuningBlobSize;
    /**
     * Uri which points to camera tuning blob
     */
    std::string cameraTuningBlobUri;

    /**
     * Chunk size for splitting device-sent XLink packets, in bytes. A larger value could
     * increase performance, with 0 disabling chunking. A negative value won't modify the
     * device defaults - configured per protocol, currently 64*1024 for both USB and Ethernet.
     */
    int32_t xlinkChunkSize = -1;

    /**
     * SIPP (Signal Image Processing Pipeline) internal memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * By default memory is allocated in high speed CMX memory. Setting to 0 will allocate in DDR 256 kilobytes.
     * Units are bytes.
     */
    uint32_t sippBufferSize = SIPP_BUFFER_DEFAULT_SIZE;
    /**
     * SIPP (Signal Image Processing Pipeline) internal DMA memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * Memory is allocated in high speed CMX memory
     * Units are bytes.
     */
    uint32_t sippDmaBufferSize = SIPP_DMA_BUFFER_DEFAULT_SIZE;
};

DEPTHAI_SERIALIZE_EXT(GlobalProperties,
                      leonCssFrequencyHz,
                      leonMssFrequencyHz,
                      pipelineName,
                      pipelineVersion,
                      cameraTuningBlobSize,
                      cameraTuningBlobUri,
                      calibData,
                      xlinkChunkSize,
                      sippBufferSize,
                      sippDmaBufferSize);

}  // namespace dai
