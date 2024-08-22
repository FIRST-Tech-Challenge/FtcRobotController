#pragma once

#include <string>

namespace dai {

bool initialize();
bool initialize(std::string additionalInfo, bool installSignalHandler = true, void* javavm = nullptr);
bool initialize(const char* additionalInfo, bool installSignalHandler = true, void* javavm = nullptr);
bool initialize(void* javavm);

}  // namespace dai
