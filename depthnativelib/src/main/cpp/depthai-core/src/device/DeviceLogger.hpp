#pragma once

#include <spdlog/spdlog.h>

namespace dai {

class DeviceLogger : public spdlog::logger {
    using spdlog::logger::logger;

public:
    void logMessage(const LogMessage& msg){
        // Convert LogMessage to spdlog::details::log_msg
        spdlog::details::log_msg log;

        // First get spdlogLevel
        spdlog::level::level_enum spdlogLevel = spdlog::level::warn;
        switch (msg.level) {
            case dai::LogLevel::TRACE: spdlogLevel = spdlog::level::trace; break;
            case dai::LogLevel::DEBUG: spdlogLevel = spdlog::level::debug; break;
            case dai::LogLevel::INFO: spdlogLevel = spdlog::level::info; break;
            case dai::LogLevel::WARN: spdlogLevel = spdlog::level::warn; break;
            case dai::LogLevel::ERR: spdlogLevel = spdlog::level::err; break;
            case dai::LogLevel::CRITICAL: spdlogLevel = spdlog::level::critical; break;
            case dai::LogLevel::OFF: spdlogLevel = spdlog::level::off; break;
        }
        // level
        log.level = spdlogLevel;

        // mimics regular "log" method
        bool logEnabled = should_log(log.level);
        bool tracebackEnabled = tracer_.enabled();
        if (!logEnabled && !tracebackEnabled) {
            return;
        }


        // Continue with other fields
        // logger name
        log.logger_name = msg.nodeIdName;

        // time
        log.time = std::chrono::time_point<spdlog::log_clock, typename spdlog::log_clock::duration>(std::chrono::duration_cast<typename spdlog::log_clock::duration>(std::chrono::seconds(msg.time.sec) + std::chrono::nanoseconds(msg.time.nsec)));

        // color
        log.color_range_start = msg.colorRangeStart;
        log.color_range_end = msg.colorRangeEnd;

        // actual log message
        log.payload = msg.payload;

        // Call the internal log_it_ method
        log_it_(log, logEnabled, tracebackEnabled);

    }

};

} // namespace dai
