#include "depthai/pipeline/node/Script.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

Script::Script(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Script(par, nodeId, std::make_unique<Script::Properties>()) {}
Script::Script(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Script, ScriptProperties>(par, nodeId, std::move(props)),
      inputs("io", Input(*this, "", Input::Type::SReceiver, {{DatatypeEnum::Buffer, true}})),
      outputs("io", Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}})) {
    properties.scriptUri = "";
    properties.scriptName = "<script>";
    properties.processor = ProcessorType::LEON_MSS;

    setInputMapRefs(&inputs);
    setOutputMapRefs(&outputs);
}

void Script::setScriptPath(const dai::Path& path, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", path)->getRelativeUri();
    scriptPath = path;
    if(name.empty()) {
        properties.scriptName = path.u8string();
    } else {
        properties.scriptName = name;
    }
}

void Script::setScript(const std::string& script, const std::string& name) {
    std::vector<std::uint8_t> data{script.begin(), script.end()};
    properties.scriptUri = assetManager.set("__script", std::move(data))->getRelativeUri();
    scriptPath = {};
    if(name.empty()) {
        properties.scriptName = "<script>";
    } else {
        properties.scriptName = name;
    }
}

void Script::setScript(const std::vector<std::uint8_t>& data, const std::string& name) {
    properties.scriptUri = assetManager.set("__script", std::move(data))->getRelativeUri();
    scriptPath = {};
    if(name.empty()) {
        properties.scriptName = "<script>";
    } else {
        properties.scriptName = name;
    }
}

void Script::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

dai::Path Script::getScriptPath() const {
    return scriptPath;
}

std::string Script::getScriptName() const {
    return properties.scriptName;
}

ProcessorType Script::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai
