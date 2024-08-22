#include <catch2/catch_all.hpp>

#include "../../src/utility/EepromDataParser.hpp"

struct ProductDevice {
    std::string oldProductName, productName, deviceName;
};
std::vector<ProductDevice> productToDeviceNames = {
    {"OAK-D LR", "OAK-D-LR", "OAK-D-LR"},
    {"OAK-D", "OAK-D", "OAK-D"},
    {"OAK-FFC 1P PoE", "OAK-FFC-1P-POE", "OAK-FFC-1P-POE"},
    {"OAK-FFC 4P", "OAK-FFC-4P", "OAK-FFC-4P"},
    {"OAK-FFC 3P", "OAK-FFC-3P", "OAK-FFC-3P"},
    {"OAK-D CM4", "OAK-D-CM4", "OAK-D-CM4"},
    {"OAK-D SR", "OAK-D-SR", "OAK-D-SR"},
    {"OAK-D PoE AF", "OAK-D-POE-AF", "OAK-D-POE"},
    {"OAK-D CM4 PoE", "OAK-D-CM4-POE", "OAK-D-CM4-POE"},
    {"rae", "RAE", "RAE"},
    {"OAK-FFC 6P", "OAK-FFC-6P", "OAK-FFC-6P"},
    {"OAK-D Lite", "OAK-D-LITE", "OAK-D-LITE"},
    {"OAK-D S2 AF", "OAK-D-S2-AF", "OAK-D-S2"},
    {"OAK-D S2 FF", "OAK-D-S2-FF", "OAK-D-S2"},
    {"OAK-D W", "OAK-D-W", "OAK-D-W"},
    {"OAK-D Pro AF", "OAK-D-PRO-AF", "OAK-D-PRO"},
    {"OAK-D Pro FF 97", "OAK-D-PRO-FF-97", "OAK-D-PRO"},
    {"OAK-D Pro FF", "OAK-D-PRO-FF", "OAK-D-PRO"},
    {"OAK-D Pro W 97", "OAK-D-PRO-W-97", "OAK-D-PRO-W"},
    {"OAK-D Pro W OV9782", "OAK-D-PRO-W-OV9782", "OAK-D-PRO-W"},
    {"OAK-D Pro W", "OAK-D-PRO-W", "OAK-D-PRO-W"},
    {"OAK-D S2 AF", "OAK-D-S2-AF", "OAK-D-S2"},
    {"OAK-D S2 FF", "OAK-D-S2-FF", "OAK-D-S2"},
    {"OAK-D W", "OAK-D-W", "OAK-D-W"},
    {"OAK-D W 97", "OAK-D-W-97", "OAK-D-W"},
    {"OAK-D SR PoE", "OAK-D-SR-POE", "OAK-D-SR-POE"},
    {"OAK-FFC-4P PoE", "OAK-FFC-4P-POE", "OAK-FFC-4P-POE"},
    {"OAK-1 AF", "OAK-1-AF", "OAK-1"},
    {"OAK-1 Lite FF", "OAK-1-LITE-FF", "OAK-1-LITE"},
    {"OAK-1 Lite FF", "OAK-1-LITE-FF", "OAK-1-LITE"},
    {"OAK-1 Lite W", "OAK-1-LITE-W", "OAK-1-LITE-W"},
    {"OAK-1 Max", "OAK-1-MAX", "OAK-1-MAX"},
    {"OAK-1 W", "OAK-1-W", "OAK-1-W"},
    {"OAK-1 AF", "OAK-1-AF", "OAK-1"},
    {"OAK-1-FF 97", "OAK-1-FF-97", "OAK-1"},
    {"OAK-1 FF", "OAK-1-FF", "OAK-1"},
    {"OAK-1 Lite FF", "OAK-1-LITE-FF", "OAK-1-LITE"},
    {"OAK-1 Lite W", "OAK-1-LITE-W", "OAK-1-LITE-W"},
    {"OAK-1 Max", "OAK-1-MAX", "OAK-1-MAX"},
    {"OAK-1 W 97", "OAK-1-W-97", "OAK-1-W"},
    {"OAK-1 W", "OAK-1-W", "OAK-1-W"},
    {"OAK-D Pro PoE AF", "OAK-D-PRO-POE-AF", "OAK-D-PRO-POE"},
    {"OAK-D Pro PoE FF", "OAK-D-PRO-POE-FF", "OAK-D-PRO-POE"},
    {"OAK-D Pro W PoE", "OAK-D-PRO-W-POE", "OAK-D-PRO-W-POE"},
    {"OAK-D Pro PoE AF", "OAK-D-PRO-POE-AF", "OAK-D-PRO-POE"},
    {"OAK-D Pro PoE FF 97", "OAK-D-PRO-POE-FF-97", "OAK-D-PRO-POE"},
    {"OAK-D Pro PoE FF", "OAK-D-PRO-POE-FF", "OAK-D-PRO-POE"},
    {"OAK-D Pro W PoE", "OAK-D-PRO-W-POE", "OAK-D-PRO-W-POE"},
    {"OAK-D S2 PoE AF", "OAK-D-S2-POE-AF", "OAK-D-S2-POE"},
    {"OAK-D S2 PoE FF", "OAK-D-S2-POE-FF", "OAK-D-S2-POE"},
    {"OAK-D W PoE", "OAK-D-W-POE", "OAK-D-W-POE"},
    {"OAK-D Pro PoE AF", "OAK-D-PRO-POE-AF", "OAK-D-PRO-POE"},
    {"OAK-D Pro PoE FF 97", "OAK-D-PRO-POE-FF-97", "OAK-D-PRO-POE"},
    {"OAK-D Pro PoE FF", "OAK-D-PRO-POE-FF", "OAK-D-PRO-POE"},
    {"OAK-D Pro W PoE", "OAK-D-PRO-W-POE", "OAK-D-PRO-W-POE"},
    {"OAK-D Pro W PoE 97", "OAK-D-PRO-W-POE-97", "OAK-D-PRO-W-POE"},
    {"OAK-D W PoE 97", "OAK-D-W-POE-97", "OAK-D-W-POE"},
    {"OAK-1 PoE FF", "OAK-1-POE-FF", "OAK-1-POE"},
    {"OAK-1 PoE AF", "OAK-1-POE-AF", "OAK-1-POE"},
    {"OAK-1 W PoE 9782", "OAK-1-W-POE-9782", "OAK-1-W-POE"},
    {"OAK-1 W PoE", "OAK-1-W-POE", "OAK-1-W-POE"},
    {"OAK-1 W PoE 9782", "OAK-1-W-POE-9782", "OAK-1-W-POE"},
    {"OAK-1 W PoE", "OAK-1-W-POE", "OAK-1-W-POE"},
    {"OAK-1 PoE AF", "OAK-1-POE-AF", "OAK-1-POE"},
    {"OAK-1 PoE FF", "OAK-1-POE-FF", "OAK-1-POE"},

    // Special cases, resolved by modifying eeprom instead
    // {"OAK-D PoE C22", "OAK-D-POE-C22", "OAK-D-POE"},
    // {"", "OAK-D-CM4-POE-C11", "OAK-D-CM4-POE"},
    // {"", "OAK-D-CM4-POE-C24", "OAK-D-CM4-POE"},
    //{"OAK-D Pro AF PB", "OAK-D-PRO-AF-PB", "OAK-D-PRO"},
    // {"OAK-D Pro FF C13", "OAK-D-PRO-FF-C13", "OAK-D-PRO"},
    // {"", "OAK-D-PRO-FF-C17", "OAK-D-PRO"},
    // {"", "OAK-D-PRO-FF-PB-FF#1", "OAK-D-PRO"},
    // {"", "OAK-D-PRO-FF-PB-FF#2", "OAK-D-PRO"},
    // {"OAK-D Pro W C06", "OAK-D-PRO-W-C06", "OAK-D-PRO-W"},
    // {"OAK-D Pro FF", "OAK-D-PRO-W-C16", "OAK-D-PRO-W"},
    // {"", "OAK-D-W-C15", "OAK-D-W"},
    // {"", "OAK-D-PRO-W-DEV", "OAK-D-PRO-W-DEV"},
    // {"", "OAK-1-LITE-C05", "OAK-1-LITE"},
    // {"OAK-1 AF", "OAK-1-AF-C20", "OAK-1"},
    // {"OAK-D Pro PoE AF C18", "OAK-D-PRO-POE-AF-C18", "OAK-D-PRO-POE-C18"},
    // {"OAK-D Pro W PoE C01", "OAK-D-PRO-W-POE-C01", "OAK-D-PRO-W-POE-C01"},

};

TEST_CASE("parsing") {
    for(size_t i = 0; i < productToDeviceNames.size(); i++) {
        INFO("Testing with iteration number: " << i);

        const auto& pd = productToDeviceNames[i];
        dai::EepromData data;
        data.productName = pd.productName;

        // Test parsing of device name
        REQUIRE(pd.deviceName == dai::utility::parseDeviceName(data));
        data.productName = pd.oldProductName;
        REQUIRE(pd.productName == dai::utility::parseProductName(data));
        REQUIRE(pd.deviceName == dai::utility::parseDeviceName(data));

        // Test parsing directly to device name
        data.deviceName = pd.deviceName;
        REQUIRE(pd.deviceName == dai::utility::parseDeviceName(data));
    }
}