#pragma once

#include "depthai-shared/common/ProcessorType.hpp"
#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify ScriptProperties options such as script uri, script name, ...
 */
struct ScriptProperties : PropertiesSerializable<Properties, ScriptProperties> {
    /**
     * Uri which points to actual script
     */
    std::string scriptUri;

    /**
     * Name of script
     */
    std::string scriptName = "<script>";

    /**
     * Which processor should execute the script
     */
    ProcessorType processor = ProcessorType::LEON_CSS;
};

DEPTHAI_SERIALIZE_EXT(ScriptProperties, scriptUri, scriptName, processor);

}  // namespace dai
