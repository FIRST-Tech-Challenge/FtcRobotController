#pragma once

#include <map>
#include <memory>
#include <vector>

#include "depthai-shared/pipeline/Assets.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

/**
 * @brief Asset is identified with string key and can store arbitrary binary data
 */
struct Asset {
    Asset() = default;
    explicit Asset(std::string k) : key(std::move(k)) {}
    const std::string key;
    std::vector<std::uint8_t> data;
    std::uint32_t alignment = 1;
    std::string getRelativeUri();
};

class AssetsMutable : public Assets {
   public:
    void set(std::string, std::uint32_t offset, std::uint32_t size, std::uint32_t alignment);
};

// Subclass which has its own storage
/**
 * @brief AssetManager can store assets and serialize
 */
class AssetManager /*: public Assets*/ {
    std::map<std::string, std::shared_ptr<Asset>> assetMap;

   public:
    /**
     * Adds all assets in an array to the AssetManager
     * @param assets Vector of assets to add
     */
    void addExisting(std::vector<std::shared_ptr<Asset>> assets);

    /**
     * Adds or overwrites an asset object to AssetManager.
     * @param asset Asset to add
     * @returns Shared pointer to asset
     */
    std::shared_ptr<dai::Asset> set(Asset asset);

    /**
     * Adds or overwrites an asset object to AssetManager with a specified key.
     * Key value will be assigned to an Asset as well
     *
     * @param key Key under which the asset should be stored
     * @param asset Asset to store
     * @returns Shared pointer to asset
     */
    std::shared_ptr<dai::Asset> set(const std::string& key, Asset asset);

    /**
     * Loads file into asset manager under specified key.
     *
     * @param key Key under which the asset should be stored
     * @param path Path to file which to load as asset
     * @param alignment [Optional] alignment of asset data in asset storage. Default is 64B
     */
    std::shared_ptr<dai::Asset> set(const std::string& key, const dai::Path& path, int alignment = 64);

    /**
     * Loads file into asset manager under specified key.
     *
     * @param key Key under which the asset should be stored
     * @param data Asset data
     * @param alignment [Optional] alignment of asset data in asset storage. Default is 64B
     * @returns Shared pointer to asset
     */
    std::shared_ptr<dai::Asset> set(const std::string& key, const std::vector<std::uint8_t>& data, int alignment = 64);
    std::shared_ptr<dai::Asset> set(const std::string& key, std::vector<std::uint8_t>&& data, int alignment = 64);

    /**
     * @returns Asset assigned to the specified key or a nullptr otherwise
     */
    std::shared_ptr<const Asset> get(const std::string& key) const;

    /**
     * @returns Asset assigned to the specified key or a nullptr otherwise
     */
    std::shared_ptr<Asset> get(const std::string& key);

    /**
     * @returns All asset stored in the AssetManager
     */
    std::vector<std::shared_ptr<const Asset>> getAll() const;

    /**
     * @returns All asset stored in the AssetManager
     */
    std::vector<std::shared_ptr<Asset>> getAll();

    /**
     * @returns Number of asset stored in the AssetManager
     */
    std::size_t size() const;

    /**
     * Removes asset with key
     * @param key Key of asset to remove
     */
    void remove(const std::string& key);

    /// Serializes
    void serialize(AssetsMutable& assets, std::vector<std::uint8_t>& assetStorage, std::string prefix = "") const;
};

}  // namespace dai
