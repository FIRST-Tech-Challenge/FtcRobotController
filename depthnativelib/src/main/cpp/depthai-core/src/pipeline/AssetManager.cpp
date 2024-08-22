#include "depthai/pipeline/AssetManager.hpp"

#include "spdlog/fmt/fmt.h"
#include "utility/spdlog-fmt.hpp"

// std
#include <fstream>

namespace dai {

std::string Asset::getRelativeUri() {
    return fmt::format("{}:{}", "asset", key);
}

std::shared_ptr<dai::Asset> AssetManager::set(Asset asset) {
    std::string key = asset.key;
    assetMap[key] = std::make_shared<Asset>(std::move(asset));
    return assetMap[key];
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, Asset asset) {
    // Rename the asset with supplied key and store
    Asset a(key);
    a.data = std::move(asset.data);
    a.alignment = asset.alignment;
    return set(std::move(a));
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, const dai::Path& path, int alignment) {
    // Load binary file at path
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if(!stream.is_open()) {
        // Throw an error
        // TODO(themarpe) - Unify exceptions into meaningful groups
        throw std::runtime_error(fmt::format("Cannot load asset, file at path {} doesn't exist.", path));
    }

    // Create an asset
    Asset binaryAsset(key);
    binaryAsset.alignment = alignment;
    binaryAsset.data = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {});
    // Store asset
    return set(std::move(binaryAsset));
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, const std::vector<std::uint8_t>& data, int alignment) {
    // Create an asset
    Asset binaryAsset(key);
    binaryAsset.alignment = alignment;
    binaryAsset.data = std::move(data);
    // Store asset
    return set(std::move(binaryAsset));
}

std::shared_ptr<dai::Asset> AssetManager::set(const std::string& key, std::vector<std::uint8_t>&& data, int alignment) {
    // Create an asset
    Asset binaryAsset(key);
    binaryAsset.alignment = alignment;
    binaryAsset.data = std::move(data);
    // Store asset
    return set(std::move(binaryAsset));
}

std::shared_ptr<const Asset> AssetManager::get(const std::string& key) const {
    if(assetMap.count(key) == 0) {
        return nullptr;
    }
    return assetMap.at(key);
}

std::shared_ptr<Asset> AssetManager::get(const std::string& key) {
    if(assetMap.count(key) == 0) {
        return nullptr;
    }
    return assetMap.at(key);
}

void AssetManager::addExisting(std::vector<std::shared_ptr<Asset>> assets) {
    // make sure that key doesn't exist already
    for(const auto& asset : assets) {
        if(assetMap.count(asset->key) > 0) throw std::logic_error("An Asset with the key: " + asset->key + " already exists.");
        std::string key = asset->key;
        assetMap[key] = asset;
    }
}

std::vector<std::shared_ptr<const Asset>> AssetManager::getAll() const {
    std::vector<std::shared_ptr<const Asset>> a;
    for(const auto& kv : assetMap) {
        a.push_back(kv.second);
    }
    return a;
}

std::vector<std::shared_ptr<Asset>> AssetManager::getAll() {
    std::vector<std::shared_ptr<Asset>> a;
    for(const auto& kv : assetMap) {
        a.push_back(kv.second);
    }
    return a;
}

std::size_t AssetManager::size() const {
    return assetMap.size();
}

void AssetManager::remove(const std::string& key) {
    assetMap.erase(key);
}

void AssetManager::serialize(AssetsMutable& mutableAssets, std::vector<std::uint8_t>& storage, std::string prefix) const {
    using namespace std;

    for(auto& kv : assetMap) {
        auto& a = *kv.second;

        // calculate additional bytes needed to offset to alignment
        int toAdd = 0;
        if(a.alignment > 1 && storage.size() % a.alignment != 0) {
            toAdd = a.alignment - (storage.size() % a.alignment);
        }

        // calculate offset
        std::uint32_t offset = static_cast<uint32_t>(storage.size()) + toAdd;

        // Add alignment bytes
        storage.resize(storage.size() + toAdd);

        // copy data
        storage.insert(storage.end(), a.data.begin(), a.data.end());

        // Add to map the currently added asset
        mutableAssets.set(prefix + a.key, offset, static_cast<uint32_t>(a.data.size()), a.alignment);
    }
}

void AssetsMutable::set(std::string key, std::uint32_t offset, std::uint32_t size, std::uint32_t alignment) {
    AssetInternal internal = {};
    internal.offset = offset;
    internal.size = size;
    internal.alignment = alignment;
    map[key] = internal;
}

}  // namespace dai
