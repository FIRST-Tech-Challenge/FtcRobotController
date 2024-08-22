#include <depthai/device/Version.hpp>

// std
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace dai {

Version::Version(const std::string& v) : versionMajor(0), versionMinor(0), versionPatch(0), buildInfo{""} {
    // Parse string
    char buffer[256]{0};
    if(std::sscanf(v.c_str(), "%u.%u.%u+%255s", &versionMajor, &versionMinor, &versionPatch, buffer) != 4) {
        if(std::sscanf(v.c_str(), "%u.%u.%u", &versionMajor, &versionMinor, &versionPatch) != 3) {
            throw std::runtime_error("Cannot parse version: " + v);
        }
    } else {
        buildInfo = std::string{buffer};
    }
}

Version::Version(unsigned vmajor, unsigned vminor, unsigned vpatch) : versionMajor(vmajor), versionMinor(vminor), versionPatch(vpatch), buildInfo{""} {}

Version::Version(unsigned vmajor, unsigned vminor, unsigned vpatch, std::string buildInfo)
    : versionMajor(vmajor), versionMinor(vminor), versionPatch(vpatch), buildInfo(buildInfo) {}

bool Version::operator==(const Version& other) const {
    if(versionMajor == other.versionMajor && versionMinor == other.versionMinor && versionPatch == other.versionPatch && buildInfo == other.buildInfo) {
        return true;
    }
    return false;
}

bool Version::operator<(const Version& other) const {
    if(versionMajor < other.versionMajor) {
        return true;
    } else if(versionMajor == other.versionMajor) {
        if(versionMinor < other.versionMinor) {
            return true;
        } else if(versionMinor == other.versionMinor) {
            if(versionPatch < other.versionPatch) {
                return true;
            } else if(versionPatch == other.versionPatch) {
                if(!buildInfo.empty() && other.buildInfo.empty()) {
                    return true;
                }
            }
        }
    }
    return false;
}

std::string Version::toString() const {
    std::string version = std::to_string(versionMajor) + "." + std::to_string(versionMinor) + "." + std::to_string(versionPatch);
    if(!buildInfo.empty()) {
        version += "+" + buildInfo;
    }
    return version;
}

std::string Version::toStringSemver() const {
    std::string version = std::to_string(versionMajor) + "." + std::to_string(versionMinor) + "." + std::to_string(versionPatch);
    return version;
}

std::string Version::getBuildInfo() const {
    return buildInfo;
}

Version Version::getSemver() const {
    return Version(versionMajor, versionMinor, versionPatch);
}

}  // namespace dai