#pragma once

#include <algorithm>
#include <exception>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "depthai-shared/common/TensorInfo.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {

/// Support for basic OpenVINO related actions like version identification of neural network blobs,...
class OpenVINO {
   public:
    /// OpenVINO Version supported version information
    enum Version { VERSION_2020_3, VERSION_2020_4, VERSION_2021_1, VERSION_2021_2, VERSION_2021_3, VERSION_2021_4, VERSION_2022_1, VERSION_UNIVERSAL };

    /// OpenVINO Blob
    struct Blob {
        /**
         * @brief Construct a new Blob from data in memory
         *
         * @param data In memory blob
         */
        Blob(std::vector<uint8_t> data);
        /**
         * @brief Construct a new Blob by loading from a filesystem path
         *
         * @param path Filesystem path to the blob
         */
        Blob(const dai::Path& path);

        /// OpenVINO version
        Version version;
        /// Map of input names to additional information
        std::unordered_map<std::string, TensorInfo> networkInputs;
        /// Map of output names to additional information
        std::unordered_map<std::string, TensorInfo> networkOutputs;
        /// Number of network stages
        uint32_t stageCount = 0;
        /// Number of shaves the blob was compiled for
        uint32_t numShaves = 0;
        /// Number of CMX slices the blob was compiled for
        uint32_t numSlices = 0;
        /// Blob data
        std::vector<uint8_t> data;
    };

    /// Main OpenVINO version
    constexpr static const Version DEFAULT_VERSION = VERSION_2022_1;

    /**
     * @returns Supported versions
     */
    static std::vector<Version> getVersions();

    /**
     * Returns string representation of a given version
     * @param version OpenVINO version
     * @returns Name of a given version
     */
    static std::string getVersionName(Version version);

    /**
     * Creates Version from string representation.
     * Throws if not possible.
     * @param versionString Version as string
     * @returns Version object if successful
     */
    static Version parseVersionName(const std::string& versionString);

    /**
     * Returns a list of potentially supported versions for a specified blob major and minor versions.
     * @param majorVersion Major version from OpenVINO blob
     * @param minorVersion Minor version from OpenVINO blob
     * @returns Vector of potentially supported versions
     */
    static std::vector<Version> getBlobSupportedVersions(std::uint32_t majorVersion, std::uint32_t minorVersion);

    /**
     * Returns latest potentially supported version by a given blob version.
     * @param majorVersion Major version from OpenVINO blob
     * @param minorVersion Minor version from OpenVINO blob
     * @returns Latest potentially supported version
     */
    static Version getBlobLatestSupportedVersion(std::uint32_t majorVersion, std::uint32_t minorVersion);

    /**
     * Returns OpenVINO version of a given blob minor/major revision.
     * @param majorVersion Major version from OpenVINO blob
     * @param minorVersion Minor version from OpenVINO blob
     * @returns Latest potentially supported version
     */
    static Version getBlobVersion(std::uint32_t majorVersion, std::uint32_t minorVersion);

    /**
     * Checks whether two blob versions are compatible
     */
    static bool areVersionsBlobCompatible(Version v1, Version v2);

   private:
    static const std::map<std::pair<std::uint32_t, std::uint32_t>, Version> blobVersionToOpenvinoGuessMapping;
    static const std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<Version>> blobVersionToOpenvinoMapping;
};

}  // namespace dai
