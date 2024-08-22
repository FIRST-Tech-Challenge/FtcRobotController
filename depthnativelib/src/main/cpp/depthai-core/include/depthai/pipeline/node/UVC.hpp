#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/UVCProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief UVC (USB Video Class) node
 */
class UVC : public NodeCRTP<Node, UVC, UVCProperties> {
   public:
    constexpr static const char* NAME = "UVC";

   public:
    UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    UVC(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Input for image frames to be streamed over UVC
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 8, true, {{DatatypeEnum::Buffer, true}}};

    /// Set GPIO list <gpio_number, value> for GPIOs to set (on/off) at init
    void setGpiosOnInit(std::unordered_map<int, int> list);

    /// Set GPIO list <gpio_number, value> for GPIOs to set when streaming is enabled
    void setGpiosOnStreamOn(std::unordered_map<int, int> list);

    /// Set GPIO list <gpio_number, value> for GPIOs to set when streaming is disabled
    void setGpiosOnStreamOff(std::unordered_map<int, int> list);
};

}  // namespace node
}  // namespace dai
