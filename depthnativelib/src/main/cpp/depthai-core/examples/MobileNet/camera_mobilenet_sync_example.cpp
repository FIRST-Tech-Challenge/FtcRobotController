#include <cstdio>
#include <deque>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

constexpr float CAMERA_FPS = 60;

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;

    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);
    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto camOut = pipeline.create<dai::node::XLinkOut>();
    auto resultOut = pipeline.create<dai::node::XLinkOut>();

    camOut->setStreamName("preview");
    resultOut->setStreamName("resultOut");

    // ColorCamera options
    camRgb->setPreviewSize(300, 300);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(CAMERA_FPS);

    // NN input options
    nn->input.setBlocking(false);
    nn->input.setQueueSize(1);
    nn->setBlobPath(nnPath);
    // Change to 1 to observe slower consumer syncing
    nn->setNumInferenceThreads(2);

    // Link nodes CAM -> XLINK
    camRgb->preview.link(nn->input);
    camRgb->preview.link(camOut->input);
    nn->out.link(resultOut->input);

    // Common variables
    std::mutex queueMtx;
    std::deque<cv::Mat> queue;
    std::atomic<bool> running{true};

    // Worker thread
    std::thread workerThread([&running, &queueMtx, &queue, &pipeline]() {
        // Connect to device and start pipeline
        dai::Device device(pipeline);

        // Create input & output queues
        auto previewQueue = device.getOutputQueue("preview");
        auto resultQueue = device.getOutputQueue("resultOut");

        // Get initial inference result
        auto prevResult = resultQueue->get<dai::ImgDetections>();

        // statistics
        nanoseconds sumLatency{0};
        milliseconds avgLatency{0};
        int numFrames = 0;
        int nnFps = 0, lastNnFps = 0;
        int camFps = 0, lastCamFps = 0;
        auto lastTime = steady_clock::now();

        while(running) {
            // Get first passthrough and result at the same time
            auto result = resultQueue->get<dai::ImgDetections>();

            nnFps++;

            // Match up prevResults and previews
            while(running) {
                // pop the preview
                auto preview = previewQueue->get<dai::ImgFrame>();
                camFps++;

                // process
                cv::Mat frame = toMat(preview->getData(), preview->getWidth(), preview->getHeight(), 3, 1);

                // Draw all detections
                for(const auto& d : prevResult->detections) {
                    int x1 = d.xmin * frame.cols;
                    int y1 = d.ymin * frame.rows;
                    int x2 = d.xmax * frame.cols;
                    int y2 = d.ymax * frame.rows;
                    cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), cv::Scalar(255, 255, 255));
                }

                // Draw avg latency and previous fps
                cv::putText(frame, std::string("NN: ") + std::to_string(lastNnFps), cv::Point(5, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
                cv::putText(frame, std::string("Cam: ") + std::to_string(lastCamFps), cv::Point(5, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
                cv::putText(frame,
                            std::string("Latency: ") + std::to_string(avgLatency.count()),
                            cv::Point(frame.cols - 120, 20),
                            cv::FONT_HERSHEY_PLAIN,
                            1,
                            cv::Scalar(255, 0, 0));

                // Send to be displayed
                {
                    std::unique_lock<std::mutex> l(queueMtx);
                    queue.push_back(std::move(frame));
                }

                // If seq number >= next detection seq number - 1, break
                if(preview->getSequenceNum() >= result->getSequenceNum() - 1) {
                    numFrames++;
                    sumLatency = sumLatency + (steady_clock::now() - preview->getTimestamp());

                    if(steady_clock::now() - lastTime >= seconds(1)) {
                        // calculate fps
                        lastNnFps = nnFps;
                        nnFps = 0;

                        // calculate cam fps
                        lastCamFps = camFps;
                        camFps = 0;

                        // calculate latency
                        avgLatency = duration_cast<milliseconds>(sumLatency / numFrames);
                        sumLatency = nanoseconds(0);
                        numFrames = 0;

                        // reset last time
                        lastTime = steady_clock::now();
                    }

                    // break out of the loop
                    break;
                }
            }

            // Move current NN results to prev
            prevResult = result;
        }
    });

    // Display (main) thread
    using namespace std::chrono;
    cv::Mat frame;
    while(running) {
        auto t1 = steady_clock::now();
        {
            std::unique_lock<std::mutex> l(queueMtx);
            if(!queue.empty()) {
                frame = queue.front();
                queue.pop_front();
            }
        }
        if(!frame.empty()) {
            cv::imshow("frame", frame);
        }
        auto toSleep = (seconds(1) / CAMERA_FPS) - (steady_clock::now() - t1);
        if(toSleep > milliseconds(0)) {
            if(cv::waitKey(duration_cast<milliseconds>(toSleep).count()) == 'q') {
                running = false;
            }
        }
    }

    workerThread.join();
}
