//-------------------------------------------------------------------
//  Nano RPC
//  https://github.com/tdv/nanorpc
//  Created:     05.2018
//  Copyright (C) 2018 tdv
//-------------------------------------------------------------------

#ifndef __NANO_RPC_VERSION_CORE_H__
#define __NANO_RPC_VERSION_CORE_H__

// STD
#include <cstdint>
#include <type_traits>

namespace nanorpc
{
namespace version
{
namespace core
{

using protocol = std::integral_constant<std::uint32_t, 1>;
    
} // namespace core
} // namespace version
} // namespace nanorpc


#endif  // !__NANO_RPC_VERSION_CORE_H__
