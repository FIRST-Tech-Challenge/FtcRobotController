#pragma once

#include <stdexcept>

namespace dai {

struct EepromError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

}  // namespace dai