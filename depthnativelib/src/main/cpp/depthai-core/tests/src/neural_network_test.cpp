#include <catch2/catch_all.hpp>

// std
#include <atomic>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

const auto MOBILENET_BLOB_PATH = BLOB_PATH;
const auto MOBILENET_WIDTH = 300;
const auto MOBILENET_HEIGHT = 300;
const auto MOBILENET_CHANNEL = 3;
const size_t MOBILENET_DATA_SIZE = MOBILENET_WIDTH * MOBILENET_HEIGHT * MOBILENET_CHANNEL;
const auto MOBILENET_INPUT_TENSOR = "data";
const auto MOBILENET_OUTPUT_TENSOR = "detection_out";

dai::Pipeline createNeuralNetworkPipeline(bool manualBlob) {
    dai::Pipeline p;
    auto x_in = p.create<dai::node::XLinkIn>();
    auto x_out = p.create<dai::node::XLinkOut>();
    auto nn = p.create<dai::node::NeuralNetwork>();

    // Load blob
    if(manualBlob) {
        dai::OpenVINO::Blob blob(BLOB_PATH);
        nn->setBlob(std::move(blob));
    } else {
        nn->setBlobPath(BLOB_PATH);
    }
    // Set input stream
    x_in->setStreamName("input");
    // Set output stream
    x_out->setStreamName("output");

    // Link nodes
    x_in->out.link(nn->input);
    nn->out.link(x_out->input);

    return p;
}

void test(bool manualBlob) {
    using namespace std::chrono;
    using namespace std::chrono_literals;

    auto pipeline = createNeuralNetworkPipeline(manualBlob);

    dai::Device device(pipeline.getOpenVINOVersion());

    std::atomic<bool> receivedLogMessage{false};

    // no warnings should appear
    device.setLogLevel(dai::LogLevel::WARN);
    device.addLogCallback([&receivedLogMessage](dai::LogMessage msg) {
        if(msg.level >= dai::LogLevel::WARN) {
            receivedLogMessage = true;
        }
    });

    // Start pipeline and feed correct sized data in various forms (Planar BGR 300*300*3 for mobilenet)
    device.startPipeline(pipeline);

    // Iterate 10 times
    for(int i = 0; i < 10; i++) {
        // Setup messages
        dai::Buffer buffer;
        buffer.setData(std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));

        dai::NNData nndata1, nndata2;
        // Specify tensor by name
        nndata1.setLayer(MOBILENET_INPUT_TENSOR, std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));
        // Specify tensor by index
        nndata2.setLayer("", std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));

        dai::ImgFrame frame;
        frame.setWidth(MOBILENET_WIDTH);
        frame.setHeight(MOBILENET_HEIGHT);
        frame.setType(dai::ImgFrame::Type::BGR888p);
        frame.setData(std::vector<uint8_t>(MOBILENET_DATA_SIZE + i * 1024 * 10));

        int msgIndex = 0;
        for(const dai::ADatatype& msg : std::initializer_list<std::reference_wrapper<dai::ADatatype>>{buffer, nndata1, nndata2, frame}) {
            std::cout << msgIndex << "\n";
            device.getInputQueue("input")->send(msg);
            bool timedOut = false;
            auto inference = device.getOutputQueue("output")->get<dai::NNData>(1s, timedOut);
            REQUIRE(inference != nullptr);
            REQUIRE(timedOut == false);
            REQUIRE(inference->hasLayer(MOBILENET_OUTPUT_TENSOR));
            msgIndex++;
        }
    }

    // At the end test if any error messages appeared
    REQUIRE(receivedLogMessage == false);
}

TEST_CASE("Neural network node data checks - setBlobPath") {
    test(false);
}
TEST_CASE("Neural network node data checks - setBlob") {
    test(true);
}
