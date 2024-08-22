
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    using namespace std;

    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    int camId = 0;
    if(argc > 1) {
        camId = std::stoi(argv[1]);
        if(argc > 2) {
            nnPath = std::string(argv[2]);
        }
    }

    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    auto xin = pipeline.create<dai::node::XLinkIn>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    xin->setStreamName("nn_in");
    xout->setStreamName("nn_out");

    // Properties
    nn->setBlobPath(nnPath);

    xin->setMaxDataSize(300 * 300 * 3);
    xin->setNumFrames(4);

    // Linking
    xin->out.link(nn->input);
    nn->out.link(xout->input);

    // Open Webcam
    cv::VideoCapture webcam(camId);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    cv::Mat frame;
    auto in = device.getInputQueue("nn_in");
    auto detections = device.getOutputQueue("nn_out");

    while(true) {
        // data to send further
        auto tensor = std::make_shared<dai::RawBuffer>();

        // Read frame from webcam
        webcam >> frame;

        // crop and resize
        frame = resizeKeepAspectRatio(frame, cv::Size(300, 300), cv::Scalar(0));

        // transform to BGR planar 300x300
        toPlanar(frame, tensor->data);

        // tensor->data = std::vector<std::uint8_t>(frame.data, frame.data + frame.total());
        in->send(tensor);

        struct Detection {
            unsigned int label;
            float score;
            float x_min;
            float y_min;
            float x_max;
            float y_max;
        };

        vector<Detection> dets;

        auto det = detections->get<dai::NNData>();
        std::vector<float> detData = det->getFirstLayerFp16();
        if(detData.size() > 0) {
            int i = 0;
            while(detData[i * 7] != -1.0f) {
                Detection d;
                d.label = detData[i * 7 + 1];
                d.score = detData[i * 7 + 2];
                d.x_min = detData[i * 7 + 3];
                d.y_min = detData[i * 7 + 4];
                d.x_max = detData[i * 7 + 5];
                d.y_max = detData[i * 7 + 6];
                i++;
                dets.push_back(d);
            }
        }

        for(const auto& d : dets) {
            int x1 = d.x_min * frame.cols;
            int y1 = d.y_min * frame.rows;
            int x2 = d.x_max * frame.cols;
            int y2 = d.y_max * frame.rows;

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cv::Scalar(255, 255, 255));
        }

        printf("===================== %lu detection(s) =======================\n", static_cast<unsigned long>(dets.size()));
        for(unsigned det = 0; det < dets.size(); ++det) {
            printf("%5d | %6.4f | %7.4f | %7.4f | %7.4f | %7.4f\n",
                   dets[det].label,
                   dets[det].score,
                   dets[det].x_min,
                   dets[det].y_min,
                   dets[det].x_max,
                   dets[det].y_max);
        }

        cv::imshow("preview", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}
