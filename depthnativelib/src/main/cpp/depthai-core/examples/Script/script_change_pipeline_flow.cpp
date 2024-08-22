#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::ColorCamera>();
    cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    cam->setInterleaved(false);
    cam->setIspScale(2, 3);
    cam->setVideoSize(720, 720);
    cam->setPreviewSize(300, 300);

    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    cam->video.link(xoutRgb->input);

    auto script = pipeline.create<dai::node::Script>();

    auto xin = pipeline.create<dai::node::XLinkIn>();
    xin->setStreamName("in");
    xin->out.link(script->inputs["toggle"]);

    cam->preview.link(script->inputs["rgb"]);
    script->setScript(R"(
        toggle = False
        while True:
            msg = node.io['toggle'].tryGet()
            if msg is not None:
                toggle = msg.getData()[0]
                node.warn('Toggle! Perform NN inferencing: ' + str(toggle))
            frame = node.io['rgb'].get()
            if toggle:
                node.io['nn'].send(frame)
    )");

    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    nn->setBlobPath(BLOB_PATH);
    script->outputs["nn"].link(nn->input);

    auto xoutNn = pipeline.create<dai::node::XLinkOut>();
    xoutNn->setStreamName("nn");
    nn->out.link(xoutNn->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);
    auto inQ = device.getInputQueue("in");
    auto qRgb = device.getOutputQueue("rgb");
    auto qNn = device.getOutputQueue("nn");

    bool runNn = false;

    auto color = cv::Scalar(255, 127, 0);

    auto drawDetections = [color](cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
    };

    while(true) {
        auto frame = qRgb->get<dai::ImgFrame>()->getCvFrame();
        auto imgDetections = qNn->tryGet<dai::ImgDetections>();
        if(imgDetections != nullptr) {
            auto detections = imgDetections->detections;
            drawDetections(frame, detections);
        }
        std::string frameText = "NN inferencing: ";
        if(runNn) {
            frameText += "On";
        } else {
            frameText += "Off";
        }
        cv::putText(frame, frameText, cv::Point(20, 20), cv::FONT_HERSHEY_TRIPLEX, 0.7, color);
        cv::imshow("Color frame", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        } else if(key == 't') {
            if(runNn) {
                std::cout << "Disabling\n";
            } else {
                std::cout << "Enabling\n";
            }
            runNn = !runNn;
            auto buf = dai::Buffer();
            std::vector<uint8_t> messageData;
            messageData.push_back(runNn);
            buf.setData(messageData);
            inQ->send(buf);
        }
    }
}