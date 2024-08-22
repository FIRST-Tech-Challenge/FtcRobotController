#include <catch2/catch_all.hpp>

// std
#include <array>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("Bootloader Config") {
    dai::DeviceBootloader::Config config;

    // By default IPv4 is 0.0.0.0 (invalid)
    REQUIRE("0.0.0.0" == config.getIPv4());

    std::string ipv4 = "192.168.1.150";
    std::string ipv4Mask = "255.255.255.0";
    std::string ipv4Gateway = "192.168.1.1";

    config.setStaticIPv4(ipv4, ipv4Mask, ipv4Gateway);

    std::array<uint8_t, 4> ipv4InMemory = {192, 168, 1, 150};
    for(size_t i = 0; i < ipv4InMemory.size(); i++) {
        REQUIRE(ipv4InMemory[i] == reinterpret_cast<uint8_t*>(&config.network.ipv4)[i]);
    }

    REQUIRE(ipv4 == config.getIPv4());
    REQUIRE(ipv4Mask == config.getIPv4Mask());
    REQUIRE(ipv4Gateway == config.getIPv4Gateway());
    REQUIRE(config.isStaticIPV4() == true);

    std::string dns = "1.1.1.1";
    std::string dnsAlt = "8.8.8.8";

    config.setDnsIPv4(dns);

    REQUIRE(config.getDnsIPv4() == dns);
    REQUIRE(config.network.ipv4DnsAlt == 0);

    config.setDnsIPv4(dns, dnsAlt);

    REQUIRE(config.getDnsIPv4() == dns);
    REQUIRE(config.getDnsAltIPv4() == dnsAlt);

    // MAC address
    std::string mac = "FF:AA:BB:CC:00:11";
    config.setMacAddress(mac);
    // std::cout << "Orig mac address: " << mac << " len: " << mac.length() << std::endl;
    // std::cout << "Get  mac address: " << config.getMacAddress() << " len: " << config.getMacAddress().length() << std::endl;
    REQUIRE(config.getMacAddress() == mac);
}

TEST_CASE("Bootloader Config retain") {
    auto config = dai::DeviceBootloader::Config::fromJson({{"my_custom_value", "hi"}});

    // By default IPv4 is 0.0.0.0 (invalid)
    REQUIRE("0.0.0.0" == config.getIPv4());

    std::string ipv4 = "192.168.1.150";
    std::string ipv4Mask = "255.255.255.0";
    std::string ipv4Gateway = "192.168.1.1";

    config.setStaticIPv4(ipv4, ipv4Mask, ipv4Gateway);

    std::array<uint8_t, 4> ipv4InMemory = {192, 168, 1, 150};
    for(size_t i = 0; i < ipv4InMemory.size(); i++) {
        REQUIRE(ipv4InMemory[i] == reinterpret_cast<uint8_t*>(&config.network.ipv4)[i]);
    }

    REQUIRE(ipv4 == config.getIPv4());
    REQUIRE(ipv4Mask == config.getIPv4Mask());
    REQUIRE(ipv4Gateway == config.getIPv4Gateway());
    REQUIRE(config.isStaticIPV4() == true);

    std::string dns = "1.1.1.1";
    std::string dnsAlt = "8.8.8.8";

    config.setDnsIPv4(dns);

    REQUIRE(config.getDnsIPv4() == dns);
    REQUIRE(config.network.ipv4DnsAlt == 0);

    config.setDnsIPv4(dns, dnsAlt);

    REQUIRE(config.getDnsIPv4() == dns);
    REQUIRE(config.getDnsAltIPv4() == dnsAlt);

    // MAC address
    std::string mac = "FF:AA:BB:CC:00:11";
    config.setMacAddress(mac);
    // std::cout << "Orig mac address: " << mac << " len: " << mac.length() << std::endl;
    // std::cout << "Get  mac address: " << config.getMacAddress() << " len: " << config.getMacAddress().length() << std::endl;
    REQUIRE(config.getMacAddress() == mac);

    // At the end check that value is retained
    REQUIRE(config.toJson()["my_custom_value"].get<std::string>() == "hi");

    // Check roundtrip
    auto j1 = config.toJson();
    REQUIRE(config.fromJson(j1).toJson() == j1);
}
