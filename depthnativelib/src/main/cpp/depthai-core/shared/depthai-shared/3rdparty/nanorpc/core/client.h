//-------------------------------------------------------------------
//  Nano RPC
//  https://github.com/tdv/nanorpc
//  Created:     05.2018
//  Copyright (C) 2018 tdv
//-------------------------------------------------------------------

#ifndef __NANO_RPC_CORE_CLIENT_H__
#define __NANO_RPC_CORE_CLIENT_H__

// STD
#include <functional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

// C++17 backwards compatibility
#include <linb/any.hpp>
#include <tl/optional.hpp>

// NANORPC
#include "nanorpc/core/detail/pack_meta.h"
#include "nanorpc/core/exception.h"
#include "nanorpc/core/type.h"
#include "nanorpc/core/hash.h"
#include "nanorpc/version/core.h"


namespace nanorpc
{
namespace core
{

template <typename TPacker>
class client final
{
private:
    class result;

public:
    client(type::executor executor)
        : executor_{std::move(executor)}
    {
    }

    template <typename ... TArgs>
    result call(std::string name, TArgs && ... args)
    {
        return call(hash_id(name), std::forward<TArgs>(args) ... );
    }

    template <typename ... TArgs>
    result call(type::id id, TArgs && ... args)
    {
        auto data = std::make_tuple(std::forward<TArgs>(args) ... );

        packer_type packer;
        auto request = packer
                .pack(version::core::protocol::value)
                .pack(detail::pack::meta::type::request)
                .pack(id)
                .pack(data)
                .to_buffer();

        auto buffer = executor_(std::move(request));
        auto response = packer.from_buffer(std::move(buffer));

        {
            version::core::protocol::value_type protocol{};
            response = response.unpack(protocol);
            if (protocol != version::core::protocol::value)
            {
                throw exception::client{"[nanorpc::core::client::call] Unsupported protocol version \"" +
                        std::to_string(protocol) + "\"."};
            }
        }

        {
            detail::pack::meta::type type{};
            response = response.unpack(type);
            if (type != detail::pack::meta::type::response)
                throw exception::client{"[nanorpc::core::client::call] Bad response type."};
        }

        {
            detail::pack::meta::status status{};
            response = response.unpack(status);
            if (status != detail::pack::meta::status::good)
            {
                std::string message;
                response = response.unpack(message);
                throw exception::logic{message};
            }
        }

        return {std::move(response)};
    }

private:
    using packer_type = TPacker;
    using deserializer_type = typename packer_type::deserializer_type;

    type::executor executor_;

    class result final
    {
    public:
        result(result &&) noexcept = default;
        result& operator = (result &&) noexcept = default;
        ~result() noexcept = default;

        template <typename T>
        T as() const
        {
            if (!value_ && !deserializer_)
                throw exception::client{"[nanorpc::core::client::result::as] No data."};

            using Type = typename std::decay<T>::type;

            if (!value_)
            {
                 if (!deserializer_)
                     throw exception::client{"[nanorpc::core::client::result::as] No data."};

                 Type data{};
                 deserializer_->unpack(data);

                 value_ = std::move(data);
                 deserializer_.reset();
            }

            return linb::any_cast<Type>(*value_);
        }

        template <typename T>
        operator T () const
        {
            return as<T>();
        }

    private:
        template <typename>
        friend class client;

        mutable tl::optional<deserializer_type> deserializer_;
        mutable tl::optional<linb::any> value_;

        result(deserializer_type deserializer)
            : deserializer_{std::move(deserializer)}
        {
        }

        result(result const &) = delete;
        result& operator = (result const &) = delete;
    };
};

}   // namespace core
}   // namespace nanorpc

#endif  // !__NANO_RPC_CORE_CLIENT_H__
