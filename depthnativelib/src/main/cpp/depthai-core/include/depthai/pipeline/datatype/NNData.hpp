#pragma once

#include <chrono>
#include <limits>
#include <unordered_map>
#include <vector>

#include "Buffer.hpp"
#include "depthai-shared/datatype/RawNNData.hpp"

namespace dai {

/**
 * NNData message. Carries tensors and their metadata
 */
class NNData : public Buffer {
    static constexpr int DATA_ALIGNMENT = 64;
    std::shared_ptr<RawBuffer> serialize() const override;
    RawNNData& rawNn;

    // store the data
    // uint8_t
    std::unordered_map<std::string, std::vector<std::uint8_t>> u8Data;
    // FP16
    std::unordered_map<std::string, std::vector<std::uint16_t>> fp16Data;

   public:
    /**
     * Construct NNData message.
     */
    NNData();
    explicit NNData(std::shared_ptr<RawNNData> ptr);
    virtual ~NNData() = default;

    // Expose
    // uint8_t
    /**
     * Set a layer with datatype U8.
     * @param name Name of the layer
     * @param data Data to store
     */
    NNData& setLayer(const std::string& name, std::vector<std::uint8_t> data);

    /**
     * Set a layer with datatype U8. Integers are cast to bytes.
     * @param name Name of the layer
     * @param data Data to store
     */
    NNData& setLayer(const std::string& name, const std::vector<int>& data);

    // fp16
    /**
     * Set a layer with datatype FP16. Float values are converted to FP16.
     * @param name Name of the layer
     * @param data Data to store
     */
    NNData& setLayer(const std::string& name, std::vector<float> data);

    /**
     * Set a layer with datatype FP16. Double values are converted to FP16.
     * @param name Name of the layer
     * @param data Data to store
     */
    NNData& setLayer(const std::string& name, std::vector<double> data);

    // getters
    /**
     * @returns Names of all layers added
     */
    std::vector<std::string> getAllLayerNames() const;

    /**
     * @returns All layers and their information
     */
    std::vector<TensorInfo> getAllLayers() const;

    /**
     * Retrieve layers tensor information
     * @param name Name of the layer
     * @param[out] tensor Outputs tensor information of that layer
     * @returns True if layer exists, false otherwise
     */
    bool getLayer(const std::string& name, TensorInfo& tensor) const;

    /**
     * Checks if given layer exists
     * @param name Name of the layer
     * @returns True if layer exists, false otherwise
     */
    bool hasLayer(const std::string& name) const;

    /**
     * Retrieve datatype of a layers tensor
     * @param name Name of the layer
     * @param[out] datatype Datatype of layers tensor
     * @returns True if layer exists, false otherwise
     */
    bool getLayerDatatype(const std::string& name, TensorInfo::DataType& datatype) const;

    // uint8
    /**
     * Convenience function to retrieve U8 data from layer
     * @param name Name of the layer
     * @returns U8 binary data
     */
    std::vector<std::uint8_t> getLayerUInt8(const std::string& name) const;

    // fp16
    /**
     * Convenience function to retrieve float values from layers FP16 tensor
     * @param name Name of the layer
     * @returns Float data
     */
    std::vector<float> getLayerFp16(const std::string& name) const;

    // int32
    /**
     * Convenience function to retrieve INT32 values from layers tensor
     * @param name Name of the layer
     * @returns INT32 data
     */
    std::vector<std::int32_t> getLayerInt32(const std::string& name) const;

    // first layer
    /**
     * Convenience function to retrieve U8 data from first layer
     * @returns U8 binary data
     */
    std::vector<std::uint8_t> getFirstLayerUInt8() const;

    /**
     * Convenience function to retrieve float values from first layers FP16 tensor
     * @returns Float data
     */
    std::vector<float> getFirstLayerFp16() const;

    /**
     * Convenience function to retrieve INT32 values from first layers tensor
     * @returns INT32 data
     */
    std::vector<std::int32_t> getFirstLayerInt32() const;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    NNData& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    NNData& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    NNData& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
