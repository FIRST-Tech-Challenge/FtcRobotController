#pragma once


// STD
#include <cassert>
#include <cstdint>
#include <iomanip>
#include <istream>
#include <iterator>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

// NANORPC
#include "nanorpc/core/detail/config.h"
#include "nanorpc/core/exception.h"
#include "nanorpc/core/type.h"

// nlohmann
#include "nlohmann/json.hpp"

namespace nanorpc
{
namespace packer
{  

class nlohmann_json final
{
private:
    class serializer;
    class deserializer;

public:
    using serializer_type = serializer;
    using deserializer_type = deserializer;

    template <typename T>
    serializer pack(T const &value)
    {
        return serializer{}.pack(value);
    }

    deserializer from_buffer(core::type::buffer buffer)
    {
        return deserializer{std::move(buffer)};
    }

private:
    class serializer final
    {
    public:
        serializer(serializer &&) noexcept = default;
        serializer& operator = (serializer &&) noexcept = default;
        ~serializer() noexcept = default;

        template <typename T>
        serializer pack(T const &value)
        {
            pack_value(value);
            return std::move(*this);
        }

        core::type::buffer to_buffer()
        {
            assert(data.size() > 0 && "Empty stream.");
            if (data.empty())
                throw core::exception::packer{"[nanorpc::packer::nlohmann_json::serializer::to_buffer] Empty data."};

            //core::type::buffer buf = nlohmann::json::to_msgpack(data);
            nlohmann::json j = data;
            auto str = j.dump();
            core::type::buffer buf(str.begin(), str.end());

            return buf;
        }

    private:

        // data storage       
        std::vector< nlohmann::json > data;

        friend class nlohmann_json;
        serializer() = default;

        serializer(serializer const &) = delete;
        serializer& operator = (serializer const &) = delete;

        template <typename T>
        void pack_value(T const &value)
        {
            data.push_back(value);
        }

    };

    class deserializer final
    {
    public:
        deserializer(deserializer &&) noexcept = default;
        deserializer& operator = (deserializer &&) noexcept = default;
        ~deserializer() noexcept = default;

        deserializer(core::type::buffer buffer)
        {
            nlohmann::from_json(nlohmann::json::parse(buffer), data);
        }

        template <typename T>
        deserializer unpack(T &value)
        {
            assert(data.size() > 0 && "Empty stream.");
            if (data.empty())
                throw core::exception::packer{"[nanorpc::packer::nlohmann_json::deserializer] Empty stream."};

            unpack_value(value);
            return std::move(*this);
        }

    private:

        // data storage
        std::vector< nlohmann::json > data;

        friend class nlohmann_json;

        deserializer(deserializer const &) = delete;
        deserializer& operator = (deserializer const &) = delete;

        template <typename T>
        void unpack_value(T &value)
        {
            nlohmann::from_json(data.front(), value);
            data.erase(data.begin());
        }
        
    };

};

}   // namespace packer
}   // namespace nanorpc
