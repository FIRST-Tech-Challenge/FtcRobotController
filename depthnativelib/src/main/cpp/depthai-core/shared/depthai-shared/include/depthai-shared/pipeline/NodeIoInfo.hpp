#pragma once

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// NodeIo informations such as name, type, ...
struct NodeIoInfo {
    enum class Type { MSender, SSender, MReceiver, SReceiver };

    std::string group;
    std::string name;
    Type type = Type::SReceiver;
    bool blocking = true;
    int queueSize = 8;
    bool waitForMessage = false;
    uint32_t id;
};

DEPTHAI_SERIALIZE_EXT(NodeIoInfo, group, name, type, blocking, queueSize, waitForMessage, id);

}  // namespace dai
