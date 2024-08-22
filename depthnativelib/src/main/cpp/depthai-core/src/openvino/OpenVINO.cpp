
#include "depthai/openvino/OpenVINO.hpp"

#include <algorithm>
#include <exception>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "BlobReader.hpp"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

namespace dai {

// Definition
constexpr OpenVINO::Version OpenVINO::DEFAULT_VERSION;

// static member init
// {{major, minor}, 'guessed openvino version to support it'}
// major and minor represent openvino NN blob version information
const std::map<std::pair<std::uint32_t, std::uint32_t>, OpenVINO::Version> OpenVINO::blobVersionToOpenvinoGuessMapping = {
    {{5, 0}, OpenVINO::VERSION_2020_3},
    {{6, 0}, OpenVINO::VERSION_2022_1},
    {{2020, 3}, OpenVINO::VERSION_2020_3},
    {{2020, 4}, OpenVINO::VERSION_2020_4},
    {{2021, 1}, OpenVINO::VERSION_2021_1},
    {{2021, 2}, OpenVINO::VERSION_2021_2},
    {{2021, 3}, OpenVINO::VERSION_2021_3},
    {{2021, 4}, OpenVINO::VERSION_2021_4},
    {{2022, 1}, OpenVINO::VERSION_2022_1},

};

const std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<OpenVINO::Version>> OpenVINO::blobVersionToOpenvinoMapping = {
    {{5, 0}, {OpenVINO::VERSION_2020_3, OpenVINO::VERSION_UNIVERSAL}},
    {{6, 0},
     {OpenVINO::VERSION_2020_4,
      OpenVINO::VERSION_2021_1,
      OpenVINO::VERSION_2021_2,
      OpenVINO::VERSION_2021_3,
      OpenVINO::VERSION_2021_4,
      OpenVINO::VERSION_2022_1,
      OpenVINO::VERSION_UNIVERSAL}},
    {{2020, 3}, {OpenVINO::VERSION_2020_3, OpenVINO::VERSION_UNIVERSAL}},
    {{2020, 4}, {OpenVINO::VERSION_2020_4, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 1}, {OpenVINO::VERSION_2021_1, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 2}, {OpenVINO::VERSION_2021_2, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 3}, {OpenVINO::VERSION_2021_3, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 4}, {OpenVINO::VERSION_2021_4, OpenVINO::VERSION_UNIVERSAL}},
    {{2022, 1}, {OpenVINO::VERSION_2022_1, OpenVINO::VERSION_UNIVERSAL}},

};

std::vector<OpenVINO::Version> OpenVINO::getVersions() {
    return {OpenVINO::VERSION_2020_3,
            OpenVINO::VERSION_2020_4,
            OpenVINO::VERSION_2021_1,
            OpenVINO::VERSION_2021_2,
            OpenVINO::VERSION_2021_3,
            OpenVINO::VERSION_2021_4,
            OpenVINO::VERSION_2022_1};
}

std::string OpenVINO::getVersionName(OpenVINO::Version version) {
    switch(version) {
        case OpenVINO::VERSION_2020_3:
            return "2020.3";
        case OpenVINO::VERSION_2020_4:
            return "2020.4";
        case OpenVINO::VERSION_2021_1:
            return "2021.1";
        case OpenVINO::VERSION_2021_2:
            return "2021.2";
        case OpenVINO::VERSION_2021_3:
            return "2021.3";
        case OpenVINO::VERSION_2021_4:
            return "2021.4";
        case OpenVINO::VERSION_2022_1:
            return "2022.1";
        case OpenVINO::VERSION_UNIVERSAL:
            return "universal";
    }
    throw std::logic_error("OpenVINO - Unknown version enum specified");
}

OpenVINO::Version OpenVINO::parseVersionName(const std::string& versionString) {
    auto versions = getVersions();
    for(const auto& v : versions) {
        if(versionString == getVersionName(v)) {
            return v;
        }
    }
    throw std::logic_error("OpenVINO - Cannot parse version name: " + versionString);
}

std::vector<OpenVINO::Version> OpenVINO::getBlobSupportedVersions(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    std::pair<std::uint32_t, std::uint32_t> blobVersion;
    blobVersion.first = majorVersion;
    blobVersion.second = minorVersion;

    if(blobVersionToOpenvinoMapping.count(blobVersion) > 0) {
        return blobVersionToOpenvinoMapping.at(blobVersion);
    }
    return {};
}

OpenVINO::Version OpenVINO::getBlobVersion(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    std::pair<std::uint32_t, std::uint32_t> blobVersion;
    blobVersion.first = majorVersion;
    blobVersion.second = minorVersion;

    return blobVersionToOpenvinoGuessMapping.at(blobVersion);
}

OpenVINO::Version OpenVINO::getBlobLatestSupportedVersion(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    (void)majorVersion;
    (void)minorVersion;
    return OpenVINO::VERSION_UNIVERSAL;
}

bool OpenVINO::areVersionsBlobCompatible(OpenVINO::Version v1, OpenVINO::Version v2) {
    // Universal check
    if(v1 == VERSION_UNIVERSAL || v2 == VERSION_UNIVERSAL) {
        return true;
    }

    // Classic check
    // Check all blob versions
    for(const auto& kv : blobVersionToOpenvinoMapping) {
        bool v1Found = false;
        bool v2Found = false;

        // Check if both openvino versions are in same blob version
        for(const auto& el : blobVersionToOpenvinoMapping.at(kv.first)) {
            if(el == v1) v1Found = true;
            if(el == v2) v2Found = true;
        }

        if(v1Found && v2Found) {
            // if both were found, return true
            return true;
        } else if(!v1Found && !v2Found) {
            // If both weren't found, continue
            continue;
        } else {
            // If one was found but other wasn't, return false
            return false;
        }
    }

    // If versions weren't matched up in any of the above cases, log an error and return false
    logger::error("OpenVINO - version compatibility check with invalid values or unknown blob version");
    return false;
}

static void blobInit(OpenVINO::Blob& blob, std::vector<uint8_t> data) {
    blob.data = std::move(data);
    BlobReader reader;
    reader.parse(blob.data);
    blob.networkInputs = reader.getNetworkInputs();
    blob.networkOutputs = reader.getNetworkOutputs();
    blob.stageCount = reader.getStageCount();
    blob.numShaves = reader.getNumberOfShaves();
    blob.numSlices = reader.getNumberOfSlices();
    blob.version = OpenVINO::getBlobVersion(reader.getVersionMajor(), reader.getVersionMinor());
}

OpenVINO::Blob::Blob(std::vector<uint8_t> data) {
    blobInit(*this, std::move(data));
}

OpenVINO::Blob::Blob(const dai::Path& path) {
    // Load binary file at path
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if(!stream.is_open()) {
        // Throw an error
        // TODO(themarpe) - Unify exceptions into meaningful groups
        throw std::runtime_error(fmt::format("Cannot load blob, file at path {} doesn't exist.", path));
    }
    blobInit(*this, std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {}));
}

}  // namespace dai
