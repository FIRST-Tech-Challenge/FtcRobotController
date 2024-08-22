//-------------------------------------------------------------------
//  Nano RPC
//  https://github.com/tdv/nanorpc
//  Created:     05.2018
//  Copyright (C) 2018 tdv
//-------------------------------------------------------------------

#ifndef __NANO_RPC_CORE_EXCEPTION_H__
#define __NANO_RPC_CORE_EXCEPTION_H__

// STD
#include <iostream>
#include <stdexcept>
#include <string>

#define NANORPC_EXCEPTION_DECL(class_, base_) \
    class class_ \
        : public base_ \
    { \
    public: \
        using base_type = base_; \
        using base_type :: base_type; \
    };

#define NANORPC_EXCEPTION_DECL_WITH_NAMESPACE(namespace_, class_, base_) \
    namespace namespace_ \
    { \
        NANORPC_EXCEPTION_DECL(class_, base_) \
    }

namespace nanorpc
{
namespace core
{
namespace exception
{
    

NANORPC_EXCEPTION_DECL(nanorpc, std::runtime_error)
NANORPC_EXCEPTION_DECL(packer, nanorpc)
NANORPC_EXCEPTION_DECL(logic, nanorpc)
NANORPC_EXCEPTION_DECL(transport, nanorpc)
NANORPC_EXCEPTION_DECL(client, transport)
NANORPC_EXCEPTION_DECL(server, transport)

inline std::string to_string(std::exception const &e)
{
    std::string message = e.what();

    try
    {
        std::rethrow_if_nested(e);
    }
    catch (std::exception const &ex)
    {
        message += "\n";
        message += "\t";
        message += to_string(ex);
    }

    return message;
}

inline void default_error_handler(std::exception_ptr e)
{
    if (!e)
        return;

    try
    {
        std::rethrow_exception(e);
    }
    catch (std::exception const &ex)
    {
        std::cerr << "NanoRPC exception message: " << to_string(ex) << std::endl;
    }
}


}   // namespace exception   
}   // namespace core
}   // namespace nanorpc


#endif  // !__NANO_RPC_CORE_EXCEPTION_H__
