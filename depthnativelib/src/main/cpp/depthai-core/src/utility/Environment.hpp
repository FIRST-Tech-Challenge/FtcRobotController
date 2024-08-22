#pragma once

#include <string>
#include <spdlog/logger.h>

namespace dai
{
namespace utility
{

std::string getEnv(const std::string& var);
std::string getEnv(const std::string& var, spdlog::logger& logger);


} // namespace utility
} // namespace dai
