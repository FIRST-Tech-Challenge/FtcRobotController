#pragma once

#include <spdlog/fmt/fmt.h>

#include "depthai/utility/Path.hpp"

namespace dai {
namespace utility {
static constexpr char path_convert_err[] = "<Unicode path not convertible>";
}
}  // namespace dai

template <>
struct fmt::formatter<dai::Path> : formatter<std::string> {
    // https://fmt.dev/latest/api.html#formatting-user-defined-types
    // https://fmt.dev/latest/syntax.html#format-specification-mini-language
    template <typename FormatContext>
    auto format(const dai::Path& p, FormatContext& ctx) {
        std::string output;
        try {
            output = p.string();
        } catch(const std::exception&) {
            output = dai::utility::path_convert_err;
        }
        return formatter<std::string>::format(output, ctx);
    }
};
