#include "utility/EepromDataParser.hpp"

namespace dai {
namespace utility {

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = s.find(delimiter);

    while(end != std::string::npos) {
        tokens.push_back(s.substr(start, end - start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    tokens.push_back(s.substr(start, end));

    return tokens;
}

std::string parseProductName(EepromData eeprom, EepromData eepromFactory) {
    std::string productName;
    if((productName = eepromFactory.productName).empty()) {
        if((productName = eeprom.productName).empty()) {
            productName = eeprom.boardName;
        }
    }

    // Convert to device naming from display/product naming
    std::transform(productName.begin(), productName.end(), productName.begin(), [](int c) { return std::toupper(c); });
    std::replace(productName.begin(), productName.end(), ' ', '-');

    // Handle some known legacy cases
    if(productName == "BW1098OBC") {
        productName = "OAK-D";
    } else if(productName == "DM2097") {
        productName = "OAK-D-CM4-POE";
    } else if(productName == "BW1097") {
        productName = "OAK-D-CM3";
    }

    return productName;
}

std::string parseDeviceName(EepromData eeprom, EepromData eepromFactory) {
    std::string deviceName;
    if((deviceName = eepromFactory.deviceName).empty()) {
        if((deviceName = eeprom.deviceName).empty()) {
            // Constant skuDiff array
            const std::array<std::string, 5> skuDiff = {"AF", "FF", "97", "9782", "OV9782"};

            // Parse productName and use that to generate device name
            deviceName = parseProductName(eeprom, eepromFactory);

            // Regenerate from tokens
            auto tokens = split(deviceName, '-');
            deviceName = "";
            for(unsigned int i = 0; i < tokens.size(); i++) {
                const auto& token = tokens[i];

                // check if token has to be removed
                bool skipToken = false;
                for(const auto& d : skuDiff) {
                    if(token == d) {
                        skipToken = true;
                        break;
                    }
                }
                if(skipToken) continue;

                if(i != 0) deviceName += "-";
                deviceName += token;
            }

            // Return generated deviceName
            return deviceName;
        }
    }

    return deviceName;
}

}  // namespace utility
}  // namespace dai
