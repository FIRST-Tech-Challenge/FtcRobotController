#pragma once

#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for NeuralNetwork such as blob path, ...
 */
struct NeuralNetworkProperties : PropertiesSerializable<Properties, NeuralNetworkProperties> {
    /**
     * Blob binary size in bytes
     */
    tl::optional<std::uint32_t> blobSize;
    /**
     * Uri which points to blob
     */
    std::string blobUri;
    /**
     * Number of available output tensors in pool
     */
    std::uint32_t numFrames = 8;
    /**
     * Number of threads to create for running inference. 0 = auto
     */
    std::uint32_t numThreads = 0;
    /**
     * Number of NCE (Neural Compute Engine) per inference thread. 0 = auto
     */
    std::uint32_t numNCEPerThread = 0;
};

DEPTHAI_SERIALIZE_EXT(NeuralNetworkProperties, blobSize, blobUri, numFrames, numThreads, numNCEPerThread);

}  // namespace dai
