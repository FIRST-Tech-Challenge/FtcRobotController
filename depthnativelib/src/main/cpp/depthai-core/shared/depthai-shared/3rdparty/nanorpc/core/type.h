//-------------------------------------------------------------------
//  Nano RPC
//  https://github.com/tdv/nanorpc
//  Created:     05.2018
//  Copyright (C) 2018 tdv
//-------------------------------------------------------------------

#ifndef __NANO_RPC_CORE_TYPE_H__
#define __NANO_RPC_CORE_TYPE_H__

// STD
#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace nanorpc
{
namespace core
{
namespace type
{


using id = std::uint64_t;
using buffer = std::vector<std::uint8_t>;
using executor = std::function<buffer (buffer)>;
using executor_map = std::map<std::string, executor>;
using error_handler = std::function<void (std::exception_ptr)>;


}   // namespace type
}   // namespace core
}   // namespace nanorpc
    

#endif  // !__NANO_RPC_CORE_TYPE_H__
