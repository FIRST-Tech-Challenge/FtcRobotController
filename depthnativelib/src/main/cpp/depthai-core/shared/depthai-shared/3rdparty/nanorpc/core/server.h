//-------------------------------------------------------------------
//  Nano RPC
//  https://github.com/tdv/nanorpc
//  Created:     05.2018
//  Copyright (C) 2018 tdv
//-------------------------------------------------------------------

#ifndef __NANO_RPC_CORE_SERVER_H__
#define __NANO_RPC_CORE_SERVER_H__

// STD
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <type_traits>

// C++17 backwards compatibility
#include <invoke_hpp/invoke.hpp>

// NANORPC
#include "nanorpc/core/detail/function_meta.h"
#include "nanorpc/core/detail/pack_meta.h"
#include "nanorpc/core/exception.h"
#include "nanorpc/core/type.h"
#include "nanorpc/core/hash.h"
#include "nanorpc/version/core.h"


namespace nanorpc::core
{

// Helper
template< class T, class U >
constexpr bool is_same_v = std::is_same<T, U>::value;

template <typename TPacker>
class server final
{
public:
    template <typename TFunc>
    void handle(std::string name, TFunc func)
    {
        handle(hash_id(name), std::move(func));
    }

    template <typename TFunc>
    void handle(type::id id, TFunc func)
    {
        if (handlers_.find(id) != std::end(handlers_))
        {
            throw std::invalid_argument{"[" + std::string{__func__ } + "] Failed to add handler. "
                    "The id \"" + std::to_string(id) + "\" already exists."};
        }

        auto wrapper = [f = std::move(func)] (deserializer_type &request, serializer_type &response)
            {
                auto func = detail::lambdaToFunction(std::move(f));
                using function_meta = detail::function_meta<decltype(func)>;
                using arguments_tuple_type = typename function_meta::arguments_tuple_type;
                arguments_tuple_type data;
                request.unpack(data);
                
                apply(std::move(func), std::move(data), response);
            };

        handlers_.emplace(std::move(id), std::move(wrapper));
    }

    type::buffer execute(type::buffer buffer)
    try
    {
        if (handlers_.empty())
            throw exception::server{"[nanorpc::core::server::execute] No handlers."};

        packer_type packer;

        auto request = packer.from_buffer(std::move(buffer));

        {
            version::core::protocol::value_type protocol{};
            request = request.unpack(protocol);
            if (protocol != version::core::protocol::value)
            {
                throw exception::server{"[nanorpc::core::server::execute] Unsupported protocol version \"" +
                        std::to_string(protocol) + "\"."};
            }
        }

        {
            detail::pack::meta::type type{};
            request = request.unpack(type);
            if (type != detail::pack::meta::type::request)
                throw exception::server{"[nanorpc::core::server::execute] Bad response type."};
        }

        type::id function_id{};
        request = request.unpack(function_id);

        auto response = packer
                .pack(version::core::protocol::value)
                .pack(detail::pack::meta::type::response);

        auto const iter = handlers_.find(function_id);
        if (iter == std::end(handlers_))
            throw exception::server{"[nanorpc::core::server::execute] Function not found."};

        try
        {
            iter->second(request, response);
        }
        catch (std::exception const &e)
        {
            response = response
                    .pack(detail::pack::meta::status::fail)
                    .pack(e.what());
        }

        return response.to_buffer();
    }
    catch (std::exception const &e)
    {
        return packer_type{}
                .pack(version::core::protocol::value)
                .pack(detail::pack::meta::type::response)
                .pack(detail::pack::meta::status::fail)
                .pack(e.what())
                .to_buffer();
    }

private:
    using packer_type = TPacker;
    using serializer_type = typename packer_type::serializer_type;
    using deserializer_type = typename packer_type::deserializer_type;
    using handler_type = std::function<void (deserializer_type &, serializer_type &)>;
    using handlers_type = std::map<type::id, handler_type>;

    handlers_type handlers_;

    template <typename TFunc, typename TArgs>
    static
    std::enable_if_t<!is_same_v<std::decay_t< decltype( invoke_hpp::apply(std::declval<TFunc>(), std::declval<TArgs>()) ) >, void>, void>
    apply(TFunc func, TArgs args, serializer_type &serializer)
    {
        auto data = invoke_hpp::apply(std::move(func), std::move(args));
        serializer = serializer.pack(detail::pack::meta::status::good);
        serializer = serializer.pack(data);
    }
    
    template <typename TFunc, typename TArgs>
    static
    std::enable_if_t<is_same_v< std::decay_t< decltype( invoke_hpp::apply(std::declval<TFunc>(), std::declval<TArgs>()) ) > , void>, void>
    apply(TFunc func, TArgs args, serializer_type &serializer)
    {
        invoke_hpp::apply(std::move(func), std::move(args));
        serializer = serializer.pack(detail::pack::meta::status::good);
    }

    
};

}   // namespace nanorpc::core

#endif  // !__NANO_RPC_CORE_SERVER_H__
