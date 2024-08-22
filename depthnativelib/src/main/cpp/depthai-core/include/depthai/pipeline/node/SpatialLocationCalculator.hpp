#pragma once

#include <depthai/pipeline/Node.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/SpatialLocationCalculatorProperties.hpp>

#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief SpatialLocationCalculator node. Calculates spatial location data on a set of ROIs on depth map.
 */
class SpatialLocationCalculator : public NodeCRTP<Node, SpatialLocationCalculator, SpatialLocationCalculatorProperties> {
   public:
    constexpr static const char* NAME = "SpatialLocationCalculator";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawSpatialLocationCalculatorConfig> rawConfig;

   public:
    SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    SpatialLocationCalculator(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    SpatialLocationCalculatorConfig initialConfig;

    /**
     * Input SpatialLocationCalculatorConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}};
    /**
     * Input message with depth data used to retrieve spatial information about detected object.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, "inputDepth", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs SpatialLocationCalculatorData message that carries spatial location results.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorData, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

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
};

}  // namespace node
}  // namespace dai
