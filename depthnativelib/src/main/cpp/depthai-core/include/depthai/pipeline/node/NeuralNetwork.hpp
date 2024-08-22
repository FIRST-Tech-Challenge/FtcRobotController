#pragma once

#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/Node.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/NeuralNetworkProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief NeuralNetwork node. Runs a neural inference on input data.
 */
class NeuralNetwork : public NodeCRTP<Node, NeuralNetwork, NeuralNetworkProperties> {
   public:
    constexpr static const char* NAME = "NeuralNetwork";

   protected:
    tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion() override;
    tl::optional<OpenVINO::Version> networkOpenvinoVersion;

   public:
    NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    NeuralNetwork(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Input message with data to be inferred upon
     * Default queue is blocking with size 5
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 5, true, {{DatatypeEnum::Buffer, true}}};

    /**
     * Outputs NNData message that carries inference results
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNData, false}}};

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Inputs mapped to network inputs. Useful for inferring from separate data sources
     * Default input is non-blocking with queue size 1 and waits for messages
     */
    InputMap inputs;

    /**
     * Passthroughs which correspond to specified input
     */
    OutputMap passthroughs;

    // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlobPath(const dai::Path& path);

    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @param blob Network blob
     */
    void setBlob(OpenVINO::Blob blob);

    /**
     * Same functionality as the setBlobPath(). Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlob(const dai::Path& path);

    /**
     * Specifies how many frames will be available in the pool
     * @param numFrames How many frames will pool have
     */
    void setNumPoolFrames(int numFrames);

    /**
     * How many threads should the node use to run the network.
     * @param numThreads Number of threads to dedicate to this node
     */
    void setNumInferenceThreads(int numThreads);

    /**
     * How many Neural Compute Engines should a single thread use for inference
     * @param numNCEPerThread Number of NCE per thread
     */
    void setNumNCEPerInferenceThread(int numNCEPerThread);

    /**
     * How many inference threads will be used to run the network
     * @returns Number of threads, 0, 1 or 2. Zero means AUTO
     */
    int getNumInferenceThreads();
    // TODO add getters for other API
};

}  // namespace node
}  // namespace dai
