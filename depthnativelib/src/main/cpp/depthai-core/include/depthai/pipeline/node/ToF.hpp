#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/ToFProperties.hpp>

#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief ToF node
 */
class ToF : public NodeCRTP<Node, ToF, ToFProperties> {
   public:
    constexpr static const char* NAME = "ToF";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawToFConfig> rawConfig;

    /**
     * Constructs ToF node.
     */
   public:
    ToF(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    ToF(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use for depth calculation.
     */
    ToFConfig initialConfig;

    /**
     * Input ToF message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ToFConfig, false}}};

    Input input{*this, "input", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    Output depth{*this, "depth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output amplitude{*this, "amplitude", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output error{*this, "error", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
};

}  // namespace node
}  // namespace dai
