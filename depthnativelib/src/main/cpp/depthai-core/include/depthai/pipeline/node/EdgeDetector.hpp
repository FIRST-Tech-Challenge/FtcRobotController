#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/EdgeDetectorProperties.hpp>

#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief EdgeDetector node. Performs edge detection using 3x3 Sobel filter
 */
class EdgeDetector : public NodeCRTP<Node, EdgeDetector, EdgeDetectorProperties> {
   public:
    constexpr static const char* NAME = "EdgeDetector";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawEdgeDetectorConfig> rawConfig;

   public:
    EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    EdgeDetector(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use for edge detection.
     */
    EdgeDetectorConfig initialConfig;

    /**
     * Input EdgeDetectorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::EdgeDetectorConfig, false}}};
    /**
     * Input image on which edge detection is performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputImage{*this, "inputImage", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs image frame with detected edges
     */
    Output outputImage{*this, "outputImage", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     */
    Output passthroughInputImage{*this, "passthroughInputImage", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] void setWaitForConfigInput(bool wait);

    /**
     * @see setWaitForConfigInput
     * @returns True if wait for inputConfig message, false otherwise
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] bool getWaitForConfigInput() const;

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Specify maximum size of output image.
     * @param maxFrameSize Maximum frame size in bytes
     */
    void setMaxOutputFrameSize(int maxFrameSize);
};

}  // namespace node
}  // namespace dai
