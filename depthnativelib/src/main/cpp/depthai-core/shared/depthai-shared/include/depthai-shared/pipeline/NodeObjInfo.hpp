#pragma once

#include <tuple>

#include "NodeIoInfo.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// NodeObj information structure
struct NodeObjInfo {
    int64_t id = -1;
    std::string name;

    std::vector<std::uint8_t> properties;

    struct IoInfoKey {
        std::size_t operator()(const std::tuple<std::string, std::string>& k) const {
            return std::hash<std::string>()(std::get<0>(k) + std::get<1>(k));
        }
    };
    std::unordered_map<std::tuple<std::string, std::string>, NodeIoInfo, IoInfoKey> ioInfo;
};

DEPTHAI_SERIALIZE_EXT(NodeObjInfo, id, name, properties, ioInfo);

}  // namespace dai
