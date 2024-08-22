#include <spdlog/spdlog.h>

#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"

int test(dai::LogLevel logLevel) {
    // Create pipeline
    dai::Pipeline pipeline;

    auto xIn = pipeline.create<dai::node::XLinkIn>();
    auto script = pipeline.create<dai::node::Script>();
    auto xOut = pipeline.create<dai::node::XLinkOut>();

    xIn->setStreamName("input");
    xOut->setStreamName("output");

    // Link xin node to script node
    xIn->out.link(script->inputs["log"]);
    script->outputs["out"].link(xOut->input);

    script->setScript(R"(
    while True:
        _ = node.io["log"].get()
        node.trace("TRACE")
        node.debug("DEBUG")
        node.info("INFO")
        node.warn("WARN")
        node.error("ERROR")
        node.critical("CRITICAL")

        message = Buffer(10)
        node.io["out"].send(message)
    )");

    dai::Device device(pipeline);

    auto in = device.getInputQueue("input");
    auto out = device.getOutputQueue("output");
    dai::Buffer message;  // Arbitrary message, used only to control flow

    device.setLogLevel(logLevel);
    device.setLogOutputLevel(logLevel);

    // -1 below is for no_error, which cannot arrive
    std::array<bool, spdlog::level::n_levels - 1> arrivedLogs;
    for(auto& level : arrivedLogs) {
        level = false;
    }
    bool testPassed = true;
    auto logLevelConverted = static_cast<typename std::underlying_type<dai::LogLevel>::type>(logLevel);
    auto callbackSink = [&testPassed, &arrivedLogs, logLevelConverted](dai::LogMessage message) {
        // Convert message to spd for easier comparison
        auto messageLevelConverted = static_cast<typename std::underlying_type<dai::LogLevel>::type>(message.level);
        REQUIRE(messageLevelConverted >= logLevelConverted);
        if(messageLevelConverted < arrivedLogs.size()) {
            arrivedLogs[messageLevelConverted] = true;
        } else {
            FAIL();
        }
    };

    device.addLogCallback(callbackSink);
    in->send(message);
    out->get();  // Wait for the device to send the log(s)
    using namespace std::chrono;
    std::this_thread::sleep_for(milliseconds(200));  // Wait for the logs to arrive

    for(int i = 0; i < arrivedLogs.size(); i++) {
        if(i < logLevelConverted) {
            REQUIRE(!arrivedLogs[i]);
        } else {
            REQUIRE(arrivedLogs[i]);
        }
    }
    device.setLogLevel(dai::LogLevel::WARN);
    device.setLogOutputLevel(dai::LogLevel::WARN);
    // Exit with success error code
    return 0;
}

TEST_CASE("TRACE") {
    test(dai::LogLevel::TRACE);
}

TEST_CASE("DEBUG") {
    test(dai::LogLevel::DEBUG);
}

TEST_CASE("INFO") {
    test(dai::LogLevel::INFO);
}

TEST_CASE("WARN") {
    test(dai::LogLevel::WARN);
}

TEST_CASE("ERROR") {
    test(dai::LogLevel::ERR);
}

TEST_CASE("CRITICAL") {
    test(dai::LogLevel::CRITICAL);
}

TEST_CASE("OFF") {
    test(dai::LogLevel::OFF);
}
