#pragma once

// std
#include <cstddef>
#include <cstdint>
#include <vector>

// libraries
#include <nop/serializer.h>
#include <nop/structure.h>
#include <nop/utility/buffer_reader.h>
#include <nop/utility/stream_writer.h>

// project
#include "NlohmannJsonCompat.hpp"

// To not require exceptions for embedded usecases.
#ifndef __has_feature           // Optional of course.
    #define __has_feature(x) 0  // Compatibility with non-clang compilers.
#endif
#if __has_feature(cxx_exceptions) || defined(__cpp_exceptions) || (defined(_MSC_VER) && defined(_CPPUNWIND)) || defined(__EXCEPTIONS)
    #define DEPTHAI_EXCEPTIONS
#endif

namespace dai {

enum class SerializationType {
    LIBNOP,
    JSON,
    JSON_MSGPACK,
};
constexpr auto static DEFAULT_SERIALIZATION_TYPE = SerializationType::LIBNOP;

namespace utility {

// JSON-msgpack serialization
template <SerializationType TYPE, typename T, std::enable_if_t<TYPE == SerializationType::JSON_MSGPACK, bool> = true>
inline bool serialize(const T& obj, std::vector<std::uint8_t>& data) {
    nlohmann::json j = obj;
    data = nlohmann::json::to_msgpack(j);
    return true;
}
template <SerializationType TYPE, typename T, std::enable_if_t<TYPE == SerializationType::JSON_MSGPACK, bool> = true>
inline bool deserialize(const std::uint8_t* data, std::size_t size, T& obj) {
    nlohmann::from_json(nlohmann::json::from_msgpack(data, data + size), obj);
    return true;
}

// JSON serialization
template <SerializationType TYPE, typename T, std::enable_if_t<TYPE == SerializationType::JSON, bool> = true>
inline bool serialize(const T& obj, std::vector<std::uint8_t>& data) {
    nlohmann::json j = obj;
    auto json = j.dump();
    data = std::vector<std::uint8_t>(reinterpret_cast<const std::uint8_t*>(json.data()), reinterpret_cast<const std::uint8_t*>(json.data()) + json.size());
    return true;
}
template <SerializationType TYPE, typename T, std::enable_if_t<TYPE == SerializationType::JSON, bool> = true>
inline bool deserialize(const std::uint8_t* data, std::size_t size, T& obj) {
    nlohmann::from_json(nlohmann::json::parse(data, data + size), obj);
    return true;
}

// NOLINTBEGIN
class VectorWriter {
   public:
    template <typename... Args>
    VectorWriter(Args&&... args) : vector{std::forward<Args>(args)...} {}
    VectorWriter(const VectorWriter&) = default;
    VectorWriter& operator=(const VectorWriter&) = default;

    nop::Status<void> Prepare(std::size_t /*size*/) {
        return {};
    }

    nop::Status<void> Write(std::uint8_t byte) {
        vector.push_back(byte);
        return ReturnStatus();
    }

    nop::Status<void> Write(const void* begin, const void* end) {
        vector.insert(vector.end(), static_cast<const std::uint8_t*>(begin), static_cast<const std::uint8_t*>(end));
        return ReturnStatus();
    }

    nop::Status<void> Skip(std::size_t padding_bytes, std::uint8_t padding_value = 0x00) {
        for(std::size_t i = 0; i < padding_bytes; i++) {
            vector.push_back(padding_value);
            auto status = ReturnStatus();
            if(!status) return status;
        }

        return {};
    }

    const std::vector<std::uint8_t>& ref() const {
        return vector;
    }
    std::vector<std::uint8_t>& ref() {
        return vector;
    }
    std::vector<std::uint8_t>&& take() {
        return std::move(vector);
    }

   private:
    nop::Status<void> ReturnStatus() {
        return {};
    }

    std::vector<std::uint8_t> vector;
};
// NOLINTEND

// libnop serialization
// If exceptions are available it throws in error cases
// Otherwise return value can be checked
template <SerializationType TYPE, typename T, std::enable_if_t<TYPE == SerializationType::LIBNOP, bool> = true>
inline bool serialize(const T& obj, std::vector<std::uint8_t>& data) {
    nop::Serializer<VectorWriter> serializer{std::move(data)};
    auto status = serializer.Write(obj);
    if(!status) {
#ifdef DEPTHAI_EXCEPTIONS
        throw std::runtime_error(status.GetErrorMessage());
#else
        return false;
#endif
    }
    data = std::move(serializer.writer().take());
    return true;
}
template <SerializationType TYPE, typename T, std::enable_if_t<TYPE == SerializationType::LIBNOP, bool> = true>
inline bool deserialize(const std::uint8_t* data, std::size_t size, T& obj) {
    nop::Deserializer<nop::BufferReader> deserializer{data, size};
    auto status = deserializer.Read(&obj);
    if(!status) {
#ifdef DEPTHAI_EXCEPTIONS
        throw std::runtime_error(status.GetErrorMessage());
#else
        return false;
#endif
    }
    return true;
}

// Serialization using enum
template <typename T>
inline bool serialize(const T& obj, std::vector<std::uint8_t>& data, SerializationType type) {
    switch(type) {
        case SerializationType::LIBNOP:
            return serialize<SerializationType::LIBNOP>(obj, data);
        case SerializationType::JSON:
            return serialize<SerializationType::JSON>(obj, data);
        case SerializationType::JSON_MSGPACK:
            return serialize<SerializationType::JSON_MSGPACK>(obj, data);
        default:
            throw std::invalid_argument("Unknown serialization type");
    };
}
template <typename T>
inline std::vector<std::uint8_t> serialize(const T& obj, SerializationType type) {
    std::vector<std::uint8_t> data;
    if(serialize(obj, data, type)) {
        return data;
    } else {
        return {};
    }
}

template <typename T>
inline bool deserialize(const std::uint8_t* data, std::size_t size, T& obj, SerializationType type) {
    switch(type) {
        case SerializationType::LIBNOP:
            return deserialize<SerializationType::LIBNOP>(data, size, obj);
        case SerializationType::JSON:
            return deserialize<SerializationType::JSON>(data, size, obj);
        case SerializationType::JSON_MSGPACK:
            return deserialize<SerializationType::JSON_MSGPACK>(data, size, obj);
        default:
            throw std::invalid_argument("Unknown serialization type");
    };
}
template <typename T>
inline bool deserialize(const std::vector<std::uint8_t>& data, T& obj, SerializationType type) {
    return deserialize(data.data(), data.size(), obj, type);
}

// Serialization using templates
template <SerializationType TYPE, typename T>
inline std::vector<std::uint8_t> serialize(const T& obj) {
    std::vector<std::uint8_t> data;
    if(serialize<TYPE>(obj, data)) {
        return data;
    } else {
        return {};
    }
}
template <SerializationType TYPE, typename T>
inline bool deserialize(const std::vector<std::uint8_t>& data, T& obj) {
    return deserialize<TYPE>(data.data(), data.size(), obj);
}

// Defaults
template <typename T>
inline bool serialize(const T& obj, std::vector<std::uint8_t>& data) {
    return serialize<DEFAULT_SERIALIZATION_TYPE>(obj, data);
}
template <typename T>
inline std::vector<std::uint8_t> serialize(const T& obj) {
    return serialize<DEFAULT_SERIALIZATION_TYPE>(obj);
}
template <typename T>
inline bool deserialize(const std::uint8_t* data, std::size_t size, T& obj) {
    return deserialize<DEFAULT_SERIALIZATION_TYPE>(data, size, obj);
}
template <typename T>
inline bool deserialize(const std::vector<std::uint8_t>& data, T& obj) {
    return deserialize<DEFAULT_SERIALIZATION_TYPE>(data, obj);
}

}  // namespace utility

// // In dai scope
// template<typename Base, typename Derived>
// struct Serializable : Base {
//     virtual void serialize(std::vector<std::uint8_t>& data) {
//         utility::serialize(static_cast<const Derived&>(*this), data);
//     }
// };

}  // namespace dai

#define DEPTHAI_DEFERRED_EXPAND(x) x
#if defined(_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL)
   // Logic using the traditional preprocessor
    // This is for suppressing false positive warnings when compiling
    // without /Zc:preprocessor
    #pragma warning(disable : 4003)
#endif

#define DEPTHAI_NLOHMANN_JSON_OPTIONAL_TO(v1) nlohmann::to_json(nlohmann_json_j[#v1], nlohmann_json_t.v1);
#define DEPTHAI_NLOHMANN_JSON_OPTIONAL_FROM(v1) \
    if(nlohmann_json_j.contains(#v1)) nlohmann_json_j[#v1].get_to(nlohmann_json_t.v1);
#define DEPTHAI_NLOHMANN_DEFINE_TYPE_OPTIONAL_NON_INTRUSIVE(Type, ...)                                              \
    inline void to_json(nlohmann::json& nlohmann_json_j, const Type& nlohmann_json_t) {                             \
        DEPTHAI_NLOHMANN_JSON_EXPAND(DEPTHAI_NLOHMANN_JSON_PASTE(DEPTHAI_NLOHMANN_JSON_OPTIONAL_TO, __VA_ARGS__))   \
    }                                                                                                               \
    inline void from_json(const nlohmann::json& nlohmann_json_j, Type& nlohmann_json_t) {                           \
        DEPTHAI_NLOHMANN_JSON_EXPAND(DEPTHAI_NLOHMANN_JSON_PASTE(DEPTHAI_NLOHMANN_JSON_OPTIONAL_FROM, __VA_ARGS__)) \
    }
#define DEPTHAI_NLOHMANN_DEFINE_TYPE_OPTIONAL_INTRUSIVE(Type, ...)                                                  \
    friend void to_json(nlohmann::json& nlohmann_json_j, const Type& nlohmann_json_t) {                             \
        DEPTHAI_NLOHMANN_JSON_EXPAND(DEPTHAI_NLOHMANN_JSON_PASTE(DEPTHAI_NLOHMANN_JSON_OPTIONAL_TO, __VA_ARGS__))   \
    }                                                                                                               \
    friend void from_json(const nlohmann::json& nlohmann_json_j, Type& nlohmann_json_t) {                           \
        DEPTHAI_NLOHMANN_JSON_EXPAND(DEPTHAI_NLOHMANN_JSON_PASTE(DEPTHAI_NLOHMANN_JSON_OPTIONAL_FROM, __VA_ARGS__)) \
    }

// Macros
#define DEPTHAI_SERIALIZE_OPTIONAL_EXT(...)                                                   \
    DEPTHAI_DEFERRED_EXPAND(DEPTHAI_NLOHMANN_DEFINE_TYPE_OPTIONAL_NON_INTRUSIVE(__VA_ARGS__)) \
    DEPTHAI_DEFERRED_EXPAND(NOP_EXTERNAL_STRUCTURE(__VA_ARGS__))

#define DEPTHAI_SERIALIZE_OPTIONAL(...)                                                   \
    DEPTHAI_DEFERRED_EXPAND(DEPTHAI_NLOHMANN_DEFINE_TYPE_OPTIONAL_INTRUSIVE(__VA_ARGS__)) \
    DEPTHAI_DEFERRED_EXPAND(NOP_EXTERNAL_STRUCTURE(__VA_ARGS__))

#define DEPTHAI_SERIALIZE_EXT(...)                                                   \
    DEPTHAI_DEFERRED_EXPAND(DEPTHAI_NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(__VA_ARGS__)) \
    DEPTHAI_DEFERRED_EXPAND(NOP_EXTERNAL_STRUCTURE(__VA_ARGS__))

#define DEPTHAI_SERIALIZE(...)                                                   \
    DEPTHAI_DEFERRED_EXPAND(DEPTHAI_NLOHMANN_DEFINE_TYPE_INTRUSIVE(__VA_ARGS__)) \
    DEPTHAI_DEFERRED_EXPAND(NOP_STRUCTURE(__VA_ARGS__))
