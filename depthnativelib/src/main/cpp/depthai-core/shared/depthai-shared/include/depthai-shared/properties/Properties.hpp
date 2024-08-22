#pragma once

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// Base Properties structure
struct Properties {
    virtual void serialize(std::vector<std::uint8_t>& data, SerializationType type) const = 0;
    virtual std::unique_ptr<Properties> clone() const = 0;
    virtual ~Properties() = default;
};

/// Serializable properties
template <typename Base, typename Derived>
struct PropertiesSerializable : Base {
    virtual void serialize(std::vector<std::uint8_t>& data, SerializationType type = SerializationType::LIBNOP) const override {
        utility::serialize(static_cast<const Derived&>(*this), data, type);
    }

    virtual std::unique_ptr<Properties> clone() const override {
        return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    }
};

}  // namespace dai