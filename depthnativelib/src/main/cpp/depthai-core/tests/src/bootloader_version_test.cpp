#include <catch2/catch_all.hpp>

// std
#include <array>

// Include depthai library
#include <depthai/depthai.hpp>

using Version = dai::DeviceBootloader::Version;

TEST_CASE("Version parsing") {
    // Okay versions
    {
        std::string str = "1.2.3";
        Version v(0, 0, 0);
        REQUIRE_NOTHROW(v = Version{str});
        REQUIRE(v.toString() == str);
    }
    {
        std::string str = "4.2.0+c137116638a96d2145ff091235de854890ed56ae";
        Version v(0, 0, 0);
        REQUIRE_NOTHROW(v = Version{str});
        REQUIRE(v.toString() == str);
    }

    // Wrong versions
    // REQUIRE_THROWS(dai::DeviceBootloader::Version("1.2.3.7+abcdef"));
    REQUIRE_THROWS(Version("1.7+abcdef"));
    REQUIRE_THROWS(Version("1..7+abcdef"));
    REQUIRE_THROWS(Version("1..7"));
    REQUIRE_THROWS(Version("1..7abcd"));
}

TEST_CASE("Version comparisons") {
    REQUIRE(Version("0.0.15+abcdef") < Version("0.0.16"));
    REQUIRE(Version("0.0.15+abcdef") < Version("0.0.15"));
    REQUIRE(Version("0.0.14") < Version("0.0.15"));
    REQUIRE(Version("0.0.14") < Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.7") < Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.15+abcdef") < Version("0.1.0"));

    REQUIRE(Version("0.0.16") > Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.15") > Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.15") > Version("0.0.14"));
    REQUIRE(Version("0.0.15+abcdef") > Version("0.0.14"));
    REQUIRE(Version("0.0.15+abcdef") > Version("0.0.7"));
    REQUIRE(Version("0.1.0") > Version("0.0.15+abcdef"));

    REQUIRE_FALSE(Version("0.0.15+abcdef") > Version("0.0.16"));
    REQUIRE_FALSE(Version("0.0.15+abcdef") > Version("0.0.15"));
    REQUIRE_FALSE(Version("0.0.14") > Version("0.0.15"));
    REQUIRE_FALSE(Version("0.0.14") > Version("0.0.15+abcdef"));
    REQUIRE_FALSE(Version("0.0.7") > Version("0.0.15+abcdef"));
    REQUIRE_FALSE(Version("0.0.15+abcdef") > Version("0.1.0"));
    REQUIRE_FALSE(Version("1.2.3") > Version("1.2.3"));

    REQUIRE_FALSE(Version("0.0.16") < Version("0.0.15+abcdef"));
    REQUIRE_FALSE(Version("0.0.15") < Version("0.0.15+abcdef"));
    REQUIRE_FALSE(Version("0.0.15") < Version("0.0.14"));
    REQUIRE_FALSE(Version("0.0.15+abcdef") < Version("0.0.14"));
    REQUIRE_FALSE(Version("0.0.15+abcdef") < Version("0.0.7"));
    REQUIRE_FALSE(Version("0.1.0") < Version("0.0.15+abcdef"));
    REQUIRE_FALSE(Version(0, 0, 18, "ee0c1c3b0f69888c9f69aa8afcc143c512bfb40d") < Version(0, 0, 12));
    REQUIRE_FALSE(Version("1.2.3") < Version("1.2.3"));

    REQUIRE(Version("0.0.15") == Version("0.0.15"));
    REQUIRE(Version("22.7.15") == Version(22, 7, 15));
    REQUIRE(Version("0.7.15") == Version(0, 7, 15));
    REQUIRE(Version("0.7.15+abc123") == Version(0, 7, 15, "abc123"));
    REQUIRE(Version("0.7.15+abc123") == Version("0.7.15+abc123"));

    REQUIRE_FALSE(Version("0.0.15") == Version("0.0.14"));
    REQUIRE_FALSE(Version("1.0.15") == Version("0.1.15"));
    REQUIRE_FALSE(Version("1.0.15") == Version("1.0.14"));
    REQUIRE_FALSE(Version("1.0.15") == Version("1.0.14+abc123"));
    REQUIRE_FALSE(Version("1.0.15") == Version("0.0.15+abc123"));
}