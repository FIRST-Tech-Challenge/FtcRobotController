#pragma once

// standard
#include <cstdint>
#include <exception>
#include <unordered_map>

// project
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

// This class represent a single asset
struct AssetView {
    std::uint8_t* data;
    std::uint32_t size;
    std::uint32_t alignment = 1;
    AssetView(std::uint8_t* d, std::uint32_t s, std::uint32_t a = 1) : data(d), size(s), alignment(a) {}
};

// This is a serializable class, which acts as readonly access to assets
class Assets {
   protected:
    std::uint8_t* pStorageStart = nullptr;

    struct AssetInternal {
        std::uint32_t offset, size, alignment;
        DEPTHAI_SERIALIZE(AssetInternal, offset, size, alignment);
    };

    // maps string to Asset
    std::unordered_map<std::string, AssetInternal> map;

   public:
    void setStorage(std::uint8_t* ps) {
        pStorageStart = ps;
    }

    bool has(const std::string& key) {
        return (map.count(key) > 0);
    }

    AssetView get(const std::string& key) {
        AssetInternal internal = map.at(key);
        return {pStorageStart + internal.offset, internal.size, internal.alignment};
    }

    std::vector<std::pair<std::string, AssetView>> getAll() {
        std::vector<std::pair<std::string, AssetView>> allAssets;
        for(const auto& kv : map) {
            allAssets.emplace_back(kv.first, get(kv.first));
        }
        return allAssets;
    }

    DEPTHAI_SERIALIZE(Assets, map);
};

}  // namespace dai
