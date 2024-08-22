#include <chrono>
#include <cstdio>
#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

int main(int argc, char** argv) {
    using namespace std;
    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setOpenVINOVersion(dai::OpenVINO::Version::VERSION_2021_4);

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    // Model expects values in FP16, as we have compiled it with `-ip FP16`
    camRgb->setFp16(true);
    camRgb->setInterleaved(false);
    camRgb->setPreviewSize(300, 300);  // NN input

    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);

    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    # Run script only once
    # Model formula:
    # output = (input - mean) / scale

    # This configuration will subtract all frame values (pixels) by 127.5
    # 0.0 .. 255.0 -> -127.5 .. 127.5
    data = NNData(2)
    data.setLayer("mean", [127.5])
    node.io['mean'].send(data)

    # This configuration will divide all frame values (pixels) by 255.0
    # -127.5 .. 127.5 -> -0.5 .. 0.5
    data = NNData(2)
    data.setLayer("scale", [255.0])
    node.io['scale'].send(data)
    )");
    // Re-use the initial values for mean/scale
    script->outputs["mean"].link(nn->inputs["mean"]);
    nn->inputs["mean"].setWaitForMessage(false);

    script->outputs["scale"].link(nn->inputs["scale"]);
    nn->inputs["scale"].setWaitForMessage(false);
    // Always wait for the new frame before starting inference
    camRgb->preview.link(nn->inputs["frame"]);

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("nn");
    nn->out.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto qNn = device.getOutputQueue("nn", 4, false);

    while(true) {
        auto inNn = qNn->get<dai::NNData>();
        // To get original frame back (0-255), we add multiply all frame values (pixels) by 255 and then add 127.5 to them.
        cv::imshow("Original Frame", fromPlanarFp16(inNn->getFirstLayerFp16(), 300, 300, 127.5, 255.0));

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
