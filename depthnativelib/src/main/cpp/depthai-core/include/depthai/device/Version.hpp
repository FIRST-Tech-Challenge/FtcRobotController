#pragma once

#include <string>

namespace dai {

/// Version structure
struct Version {
    /// Construct Version from string
    explicit Version(const std::string& v);
    /// Construct Version major, minor and patch numbers
    Version(unsigned major, unsigned minor, unsigned patch);
    /// Construct Version major, minor and patch numbers with buildInfo
    Version(unsigned major, unsigned minor, unsigned patch, std::string buildInfo);
    bool operator==(const Version& other) const;
    bool operator<(const Version& other) const;
    inline bool operator!=(const Version& rhs) const {
        return !(*this == rhs);
    }
    inline bool operator>(const Version& rhs) const {
        return rhs < *this;
    }
    inline bool operator<=(const Version& rhs) const {
        return !(*this > rhs);
    }
    inline bool operator>=(const Version& rhs) const {
        return !(*this < rhs);
    }
    /// Convert Version to string
    std::string toString() const;
    /// Convert Version to semver (no build information) string
    std::string toStringSemver() const;
    /// Get build info
    std::string getBuildInfo() const;
    /// Retrieves semver version (no build information)
    Version getSemver() const;

   private:
    unsigned versionMajor, versionMinor, versionPatch;
    std::string buildInfo;
};

}  // namespace dai