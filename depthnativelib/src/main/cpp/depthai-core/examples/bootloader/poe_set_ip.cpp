#include <chrono>
#include <iostream>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// for string delimiter
std::vector<int> split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<int> res;

    while((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(stoi(token));
    }
    res.push_back(stoi(s.substr(pos_start)));
    return res;
}

std::string checkStr(std::string str) {
    std::vector<int> v = split(str, ".");
    if(v.size() != 4) {
        std::cout << "Entered value " << str << " doesn't contain 3 dots. Value has to be in the following format: '255.255.255.255'" << std::endl;
        exit(0);
    }
    for(auto i : v) {
        if(i < 0 || 255 < i) {
            std::cout << "Entered values can't be above 255!" << std::endl;
            exit(0);
        }
    }
    return str;
}

int main(int argc, char** argv) {
    bool found = false;
    dai::DeviceInfo info;
    std::tie(found, info) = dai::DeviceBootloader::getFirstAvailableDevice();
    if(!found) {
        std::cout << "No device found to flash. Exiting." << std::endl;
        return -1;
    }

    std::cout << "Found device with name: " << info.getMxId() << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "\"1\" to set a static IPv4 address" << std::endl;
    std::cout << "\"2\" to set a dynamic IPv4 address" << std::endl;
    std::cout << "\"3\" to clear the config" << std::endl;
    auto key = std::cin.get();
    std::cout << "-------------------------------------" << std::endl;

    bool success = false;
    std::string error;
    dai::DeviceBootloader bl(info, true);
    if(key == '1' || key == '2') {
        std::string ip, mask, gateway, in;

        std::cout << "Enter IPv4: ";
        std::cin >> ip;
        checkStr(ip);

        std::cout << "Enter IPv4 Mask: ";
        std::cin >> mask;
        checkStr(mask);

        std::cout << "Enter IPv4 Gateway: ";
        std::cin >> gateway;
        checkStr(gateway);

        std::string mode = "static";
        if(key == '2') mode = "dynamic";
        std::cout << "Flashing " << mode << " IPv4 " << ip << ", mask " << mask << ", gateway " << gateway << " to the POE device. Enter 'y' to confirm. ";
        std::cin >> in;
        if(in != "y") {
            std::cout << "Flashing aborted.";
            return 0;
        }
        auto conf = dai::DeviceBootloader::Config();
        if(key == '1') {
            conf.setStaticIPv4(ip, mask, gateway);
        } else {
            conf.setDynamicIPv4(ip, mask, gateway);
        }

        std::tie(success, error) = bl.flashConfig(conf);
    } else if(key == '3') {
        std::tie(success, error) = bl.flashConfigClear();
    } else {
        std::cout << "Entered value should either be '1', '2' or '3'!";
        return 0;
    }

    if(success) {
        std::cout << "Flashing successful." << std::endl;
    } else {
        std::cout << "Flashing failed: " << error << std::endl;
    }
}
