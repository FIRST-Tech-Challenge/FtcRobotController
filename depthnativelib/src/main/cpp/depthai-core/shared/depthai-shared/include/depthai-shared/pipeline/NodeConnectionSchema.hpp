#pragma once

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Specifies a connection between nodes IOs
 */
struct NodeConnectionSchema {
    int64_t node1Id = -1;
    std::string node1OutputGroup;
    std::string node1Output;
    int64_t node2Id = -1;
    std::string node2InputGroup;
    std::string node2Input;

    bool operator==(const NodeConnectionSchema& rhs) const {
        return node1Id == rhs.node1Id && node1OutputGroup == rhs.node1OutputGroup && node1Output == rhs.node1Output && node2Id == rhs.node2Id
               && node2InputGroup == rhs.node2InputGroup && node2Input == rhs.node2Input;
    }
};

DEPTHAI_SERIALIZE_EXT(NodeConnectionSchema, node1Id, node1OutputGroup, node1Output, node2Id, node2InputGroup, node2Input);

}  // namespace dai
