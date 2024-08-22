#include "Logging.hpp"

namespace dai {

Logging::Logging() : logger("depthai", {std::make_shared<spdlog::sinks::stdout_color_sink_mt>()}) {
    // Default global logging level set to WARN; override with ENV variable 'DEPTHAI_LEVEL'
    // Taken from spdlog, to replace with DEPTHAI_LEVEL instead of SPDLOG_LEVEL
    // spdlog::cfg::load_env_levels();
    auto level = spdlog::level::warn;
    auto envLevel = utility::getEnv("DEPTHAI_LEVEL", logger);
    if(!envLevel.empty()) {
        level = parseLevel(envLevel);
    }
    logger.set_level(level);

    auto debugStr = utility::getEnv("DEPTHAI_DEBUG", logger);
    if(!debugStr.empty()) {
        // Try parsing the string as a number
        try {
            int debug{std::stoi(debugStr)};
            if(debug && (level > spdlog::level::debug)) {
                logger.set_level(spdlog::level::debug);
                logger.info("DEPTHAI_DEBUG enabled, lowered DEPTHAI_LEVEL to 'debug'");
            }
        } catch(const std::invalid_argument& e) {
            logger.warn("DEPTHAI_DEBUG value invalid: {}, should be a number (non-zero to enable)", e.what());
        }
    }
}

spdlog::level::level_enum Logging::parseLevel(std::string lvl) {
    std::transform(lvl.begin(), lvl.end(), lvl.begin(), [](char ch) { return static_cast<char>((ch >= 'A' && ch <= 'Z') ? ch + ('a' - 'A') : ch); });

    if(lvl == "trace") {
        return spdlog::level::trace;
    } else if(lvl == "debug") {
        return spdlog::level::debug;
    } else if(lvl == "info") {
        return spdlog::level::info;
    } else if(lvl == "warn") {
        return spdlog::level::warn;
    } else if(lvl == "error") {
        return spdlog::level::err;
    } else if(lvl == "off") {
        return spdlog::level::off;
    } else {
        throw std::invalid_argument(fmt::format("Cannot parse logging level: {}", lvl));
    }
}

}  // namespace dai
