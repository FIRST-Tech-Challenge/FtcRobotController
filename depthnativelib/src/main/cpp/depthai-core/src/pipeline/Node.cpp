#include "depthai/pipeline/Node.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "spdlog/fmt/fmt.h"

namespace dai {

Node::Node(const std::shared_ptr<PipelineImpl>& p, Id nodeId, std::unique_ptr<Properties> props)
    : parent(p), id(nodeId), propertiesHolder(std::move(props)), properties(*propertiesHolder) {}

tl::optional<OpenVINO::Version> Node::getRequiredOpenVINOVersion() {
    return tl::nullopt;
}

const Pipeline Node::getParentPipeline() const {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

Pipeline Node::getParentPipeline() {
    Pipeline pipeline(std::shared_ptr<PipelineImpl>{parent});
    return pipeline;
}

Properties& Node::getProperties() {
    return properties;
}

Node::Connection::Connection(Output out, Input in) {
    outputId = out.getParent().id;
    outputName = out.name;
    outputGroup = out.group;
    inputId = in.getParent().id;
    inputName = in.name;
    inputGroup = in.group;
}

bool Node::Connection::operator==(const Node::Connection& rhs) const {
    return (outputId == rhs.outputId && outputName == rhs.outputName && outputGroup == rhs.outputGroup && inputId == rhs.inputId && inputName == rhs.inputName
            && inputGroup == rhs.inputGroup);
}

std::string Node::Output::toString() const {
    if(group == "") {
        return fmt::format("{}", name);
    } else {
        return fmt::format("{}[\"{}\"]", group, name);
    }
}

std::string Node::Input::toString() const {
    if(group == "") {
        return fmt::format("{}", name);
    } else {
        return fmt::format("{}[\"{}\"]", group, name);
    }
}

std::vector<Node::Connection> Node::Output::getConnections() {
    std::vector<Node::Connection> myConnections;
    auto allConnections = parent.getParentPipeline().getConnections();
    for(const auto& conn : allConnections) {
        if(conn.outputId == parent.id && conn.outputName == name && conn.outputGroup == group) {
            myConnections.push_back(conn);
        }
    }
    return myConnections;
}

bool Node::Output::isSamePipeline(const Input& in) {
    // Check whether current output and 'in' are on same pipeline.
    // By checking parent of node
    auto outputPipeline = parent.parent.lock();
    if(outputPipeline != nullptr) {
        return (outputPipeline == in.parent.parent.lock());
    }
    return false;
}

bool Node::Output::canConnect(const Input& in) {
    return PipelineImpl::canConnect(*this, in);
}

void Node::Output::link(const Input& in) {
    // Call link of pipeline
    parent.getParentPipeline().link(*this, in);
}

void Node::Output::unlink(const Input& in) {
    // Call unlink of pipeline parents pipeline
    parent.getParentPipeline().unlink(*this, in);
}

void Node::Input::setBlocking(bool newBlocking) {
    blocking = newBlocking;
}

bool Node::Input::getBlocking() const {
    if(blocking) {
        return *blocking;
    }
    return defaultBlocking;
}

void Node::Input::setQueueSize(int size) {
    queueSize = size;
}

int Node::Input::getQueueSize() const {
    if(queueSize) {
        return *queueSize;
    }
    return defaultQueueSize;
}

void Node::Input::setWaitForMessage(bool newWaitForMessage) {
    waitForMessage = newWaitForMessage;
}

bool Node::Input::getWaitForMessage() const {
    return waitForMessage.value_or(defaultWaitForMessage);
}

void Node::Input::setReusePreviousMessage(bool reusePreviousMessage) {
    waitForMessage = !reusePreviousMessage;
}

bool Node::Input::getReusePreviousMessage() const {
    return !waitForMessage.value_or(defaultWaitForMessage);
}

const AssetManager& Node::getAssetManager() const {
    return assetManager;
}

AssetManager& Node::getAssetManager() {
    return assetManager;
}

Node::OutputMap::OutputMap(std::string name, Node::Output defaultOutput) : defaultOutput(defaultOutput), name(std::move(name)) {}
Node::OutputMap::OutputMap(Node::Output defaultOutput) : defaultOutput(defaultOutput) {}
Node::Output& Node::OutputMap::operator[](const std::string& key) {
    if(count(key) == 0) {
        // Create using default and rename with group and key
        Output output(defaultOutput);
        output.group = name;
        output.name = key;
        insert(std::make_pair(key, output));
    }
    // otherwise just return reference to existing
    return at(key);
}

Node::InputMap::InputMap(std::string name, Node::Input defaultInput) : defaultInput(defaultInput), name(std::move(name)) {}
Node::InputMap::InputMap(Node::Input defaultInput) : defaultInput(defaultInput) {}
Node::Input& Node::InputMap::operator[](const std::string& key) {
    if(count(key) == 0) {
        // Create using default and rename with group and key
        Input input(defaultInput);
        input.group = name;
        input.name = key;
        insert(std::make_pair(key, input));
    }
    // otherwise just return reference to existing
    return at(key);
}

/// Retrieves all nodes outputs
std::vector<Node::Output> Node::getOutputs() {
    std::vector<Node::Output> result;
    for(auto* x : getOutputRefs()) {
        result.push_back(*x);
    }
    return result;
}

/// Retrieves all nodes inputs
std::vector<Node::Input> Node::getInputs() {
    std::vector<Node::Input> result;
    for(auto* x : getInputRefs()) {
        result.push_back(*x);
    }
    return result;
}

/// Retrieves reference to node outputs
std::vector<Node::Output*> Node::getOutputRefs() {
    std::vector<Node::Output*> tmpOutputRefs;
    // Approximate reservation
    tmpOutputRefs.reserve(outputRefs.size() + outputMapRefs.size() * 5);
    // Add outputRefs
    for(auto& kv : outputRefs) {
        tmpOutputRefs.push_back(kv.second);
    }
    // Add outputs from Maps
    for(auto& kvMap : outputMapRefs) {
        auto*& map = kvMap.second;
        for(auto& kv : *map) {
            tmpOutputRefs.push_back(&kv.second);
        }
    }
    return tmpOutputRefs;
}

/// Retrieves reference to node outputs
std::vector<const Node::Output*> Node::getOutputRefs() const {
    std::vector<const Node::Output*> tmpOutputRefs;
    // Approximate reservation
    tmpOutputRefs.reserve(outputRefs.size() + outputMapRefs.size() * 5);
    // Add outputRefs
    for(const auto& kv : outputRefs) {
        tmpOutputRefs.push_back(kv.second);
    }
    // Add outputs from Maps
    for(const auto& kvMap : outputMapRefs) {
        const auto* const& map = kvMap.second;
        for(const auto& kv : *map) {
            tmpOutputRefs.push_back(&kv.second);
        }
    }
    return tmpOutputRefs;
}
/// Retrieves reference to node inputs
std::vector<Node::Input*> Node::getInputRefs() {
    std::vector<Node::Input*> tmpInputRefs;
    // Approximate reservation
    tmpInputRefs.reserve(inputRefs.size() + inputMapRefs.size() * 5);
    // Add inputRefs
    for(auto& kv : inputRefs) {
        tmpInputRefs.push_back(kv.second);
    }
    // Add inputs from Maps
    for(auto& kvMap : inputMapRefs) {
        auto*& map = kvMap.second;
        for(auto& kv : *map) {
            tmpInputRefs.push_back(&kv.second);
        }
    }
    return tmpInputRefs;
}

/// Retrieves reference to node inputs
std::vector<const Node::Input*> Node::getInputRefs() const {
    std::vector<const Node::Input*> tmpInputRefs;
    // Approximate reservation
    tmpInputRefs.reserve(inputRefs.size() + inputMapRefs.size() * 5);
    // Add inputRefs
    for(const auto& kv : inputRefs) {
        tmpInputRefs.push_back(kv.second);
    }
    // Add inputs from Maps
    for(const auto& kvMap : inputMapRefs) {
        const auto* const& map = kvMap.second;
        for(const auto& kv : *map) {
            tmpInputRefs.push_back(&kv.second);
        }
    }
    return tmpInputRefs;
}

void Node::setOutputRefs(std::initializer_list<Node::Output*> l) {
    for(auto& outRef : l) {
        outputRefs[outRef->name] = outRef;
    }
}
void Node::setOutputRefs(Node::Output* outRef) {
    outputRefs[outRef->name] = outRef;
}
void Node::setInputRefs(std::initializer_list<Node::Input*> l) {
    for(auto& inRef : l) {
        inputRefs[inRef->name] = inRef;
    }
}
void Node::setInputRefs(Node::Input* inRef) {
    inputRefs[inRef->name] = inRef;
}
void Node::setOutputMapRefs(std::initializer_list<Node::OutputMap*> l) {
    for(auto& outMapRef : l) {
        outputMapRefs[outMapRef->name] = outMapRef;
    }
}
void Node::setOutputMapRefs(Node::OutputMap* outMapRef) {
    outputMapRefs[outMapRef->name] = outMapRef;
}
void Node::setInputMapRefs(std::initializer_list<Node::InputMap*> l) {
    for(auto& inMapRef : l) {
        inputMapRefs[inMapRef->name] = inMapRef;
    }
}
void Node::setInputMapRefs(Node::InputMap* inMapRef) {
    inputMapRefs[inMapRef->name] = inMapRef;
}

}  // namespace dai
