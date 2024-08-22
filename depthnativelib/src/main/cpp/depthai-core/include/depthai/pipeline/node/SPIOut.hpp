#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/SPIOutProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SPIOut node. Sends messages over SPI.
 */
class SPIOut : public NodeCRTP<Node, SPIOut, SPIOutProperties> {
   public:
    constexpr static const char* NAME = "SPIOut";

    SPIOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
        : NodeCRTP<Node, SPIOut, SPIOutProperties>(par, nodeId, std::move(props)) {
        properties.busId = 0;

        setInputRefs({&input});
    }
    SPIOut(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : SPIOut(par, nodeId, std::make_unique<SPIOut::Properties>()) {}

    /**
     * Input for any type of messages to be transferred over SPI stream
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 8, true, {{DatatypeEnum::Buffer, true}}};

    /**
     * Specifies stream name over which the node will send data
     *
     * @param name Stream name
     */
    void setStreamName(std::string name) {
        properties.streamName = name;
    }

    /**
     * Specifies SPI Bus number to use
     * @param id SPI Bus id
     */
    void setBusId(int busId) {
        properties.busId = busId;
    }
};

}  // namespace node
}  // namespace dai
