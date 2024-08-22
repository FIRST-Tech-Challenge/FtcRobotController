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

// cereal
#include "cereal/cereal.hpp"
#include "cereal/archives/binary.hpp"
#include <cereal/types/vector.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp>


// Workaround for cereal not supporting serializing const char*
namespace cereal {
    
    //! Serializing for const char*
    template <class Archive, class ... Types> inline
    void CEREAL_SERIALIZE_FUNCTION_NAME( Archive & ar, const char* c_str )
    {
       ar(std::string(c_str));
    }
}

namespace nanorpc
{
namespace packer
{
  

class cereal_binary final
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
            assert(stream_ && "Empty stream.");
            if (!stream_)
                throw core::exception::packer{"[nanorpc::packer::cereal_binary::serializer::pack] Empty stream."};

            pack_value(value);
            return std::move(*this);
        }

        core::type::buffer to_buffer()
        {
            assert(stream_ && "Empty stream.");
            if (!stream_)
                throw core::exception::packer{"[nanorpc::packer::cereal_binary::serializer::to_buffer] Empty stream."};

            archive_ = nullptr;
            auto str = std::move(stream_->str());
            return {begin(str), end(str)};
        }

    private:

        using stream_type = std::stringstream;
        using stream_type_ptr = std::unique_ptr<stream_type>;

        stream_type_ptr stream_{std::make_unique<std::stringstream>()};
        std::unique_ptr<cereal::BinaryOutputArchive> archive_;

        friend class cereal_binary;
        serializer(){
            archive_ = std::make_unique<cereal::BinaryOutputArchive>(*stream_);
        }

        serializer(serializer const &) = delete;
        serializer& operator = (serializer const &) = delete;


        template <typename T>
        void pack_value(T const &value)
        {
            (*archive_)(value);
        }

        // c string specialization
        void pack_value(const char* value)
        {
            (*archive_)(std::string(value));
        }

    };

    class deserializer final
    {
    public:
        deserializer(deserializer &&) noexcept = default;
        deserializer& operator = (deserializer &&) noexcept = default;
        ~deserializer() noexcept = default;

        template <typename T>
        deserializer unpack(T &value)
        {
            assert(stream_ && "Empty stream.");
            if (!stream_)
                throw core::exception::packer{"[nanorpc::packer::plain_text::deserializer] Empty stream."};

            unpack_value(value);
            return std::move(*this);
        }

    private:

        using stream_type = std::stringstream;
        using stream_type_ptr = std::unique_ptr<stream_type>;
        stream_type_ptr stream_{std::make_unique<stream_type>()};

        std::unique_ptr<cereal::BinaryInputArchive> archive_;


        friend class cereal_binary;

        deserializer(deserializer const &) = delete;
        deserializer& operator = (deserializer const &) = delete;

        deserializer(core::type::buffer buffer)
            : stream_{std::make_unique<stream_type>(std::string{begin(buffer), end(buffer)})}
        {
            archive_ = std::make_unique<cereal::BinaryInputArchive>(*stream_);
        }

  
        template <typename T>
        void unpack_value(T &value)
        {
            (*archive_)(value);
        }

        // c string specialization
        void pack_value(char* value)
        {
            std::string val;
            (*archive_)(val);            
            strcpy(value, val.c_str());
        }

    };

};

}   // namespace packer
}   // namespace nanorpc
