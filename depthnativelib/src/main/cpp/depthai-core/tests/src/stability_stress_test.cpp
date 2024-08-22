// std
#include <chrono>
#include <csignal>
#include <iostream>
#include <unordered_map>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// #define DEPTHAI_STABILITY_TEST_SCRIPT

#ifdef DEPTHAI_STABILITY_TEST_DEBUG
    #include <opencv2/opencv.hpp>
#endif

static constexpr int RGB_FPS = 20;
static constexpr int MONO_FPS = 20;
static constexpr int ENCODER_FPS = 10;

void printSystemInformation(dai::SystemInformation info) {
    printf("Ddr used / total - %.2f / %.2f MiB\n", info.ddrMemoryUsage.used / (1024.0f * 1024.0f), info.ddrMemoryUsage.total / (1024.0f * 1024.0f));
    printf("Cmx used / total - %.2f / %.2f MiB\n", info.cmxMemoryUsage.used / (1024.0f * 1024.0f), info.cmxMemoryUsage.total / (1024.0f * 1024.0f));
    printf("LeonCss heap used / total - %.2f / %.2f MiB\n",
           info.leonCssMemoryUsage.used / (1024.0f * 1024.0f),
           info.leonCssMemoryUsage.total / (1024.0f * 1024.0f));
    printf("LeonMss heap used / total - %.2f / %.2f MiB\n",
           info.leonMssMemoryUsage.used / (1024.0f * 1024.0f),
           info.leonMssMemoryUsage.total / (1024.0f * 1024.0f));
    const auto& t = info.chipTemperature;
    printf("Chip temperature - average: %.2f, css: %.2f, mss: %.2f, upa: %.2f, dss: %.2f\n", t.average, t.css, t.mss, t.upa, t.dss);
    printf("Cpu usage - Leon CSS: %.2f %%, Leon MSS: %.2f %%\n", info.leonCssCpuUsage.average * 100, info.leonMssCpuUsage.average * 100);
}

static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

// Keyboard interrupt (Ctrl + C) detected
static std::atomic<bool> alive{true};
static void sigintHandler(int signum) {
    alive = false;
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::signal(SIGINT, &sigintHandler);

    std::string nnPath(BLOB_PATH);

    // // If path to blob specified, use that
    // if(argc > 1) {
    //     nnPath = std::string(argv[1]);
    // }

    seconds TEST_TIMEOUT{24 * 60 * 60};
    if(argc > 1) {
        TEST_TIMEOUT = seconds{stoi(argv[1])};
    }

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto ve1 = pipeline.create<dai::node::VideoEncoder>();
    auto ve2 = pipeline.create<dai::node::VideoEncoder>();
    auto ve3 = pipeline.create<dai::node::VideoEncoder>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto edgeDetectorLeft = pipeline.create<dai::node::EdgeDetector>();
    auto edgeDetectorRight = pipeline.create<dai::node::EdgeDetector>();
    auto edgeDetectorRgb = pipeline.create<dai::node::EdgeDetector>();
    auto sysLog = pipeline.create<dai::node::SystemLogger>();
#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
    auto script1 = pipeline.create<dai::node::Script>();
    auto script2 = pipeline.create<dai::node::Script>();
    auto script3 = pipeline.create<dai::node::Script>();
    auto script4 = pipeline.create<dai::node::Script>();
    auto scriptBurn = pipeline.create<dai::node::Script>();
#endif

    // TODO(themarpe) - enable specific parts separatelly, to control load
    // auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    // auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    auto ve1Out = pipeline.create<dai::node::XLinkOut>();
    auto ve2Out = pipeline.create<dai::node::XLinkOut>();
    auto ve3Out = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutEdgeLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutEdgeRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutEdgeRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutSysLog = pipeline.create<dai::node::XLinkOut>();
#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
    auto scriptOut = pipeline.create<dai::node::XLinkOut>();
    auto scriptOut2 = pipeline.create<dai::node::XLinkOut>();
#endif
    // auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    // auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();

    ve1Out->setStreamName("ve1Out");
    ve2Out->setStreamName("ve2Out");
    ve3Out->setStreamName("ve3Out");
    xoutDepth->setStreamName("depth");
    xoutNN->setStreamName("detections");
    xoutRgb->setStreamName("rgb");
    xoutSysLog->setStreamName("sysinfo");
    const auto edgeLeftStr = "edge left";
    const auto edgeRightStr = "edge right";
    const auto edgeRgbStr = "edge rgb";
    const auto edgeCfgStr = "edge cfg";
    xoutEdgeLeft->setStreamName(edgeLeftStr);
    xoutEdgeRight->setStreamName(edgeRightStr);
    xoutEdgeRgb->setStreamName(edgeRgbStr);
#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
    scriptOut->setStreamName("script");
    scriptOut2->setStreamName("script2");
#endif
    // xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    // xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    camRgb->setPreviewSize(416, 416);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(RGB_FPS);

    monoLeft->setCamera("left");
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setFps(MONO_FPS);

    monoRight->setCamera("right");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setFps(MONO_FPS);

    // Setting to 26fps will trigger error
    ve1->setDefaultProfilePreset(ENCODER_FPS, dai::VideoEncoderProperties::Profile::H264_MAIN);
    ve2->setDefaultProfilePreset(ENCODER_FPS, dai::VideoEncoderProperties::Profile::H265_MAIN);
    ve3->setDefaultProfilePreset(ENCODER_FPS, dai::VideoEncoderProperties::Profile::H264_MAIN);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // Align depth map to the perspective of RGB camera, on which inference is done
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    stereo->setOutputSize(monoLeft->getResolutionWidth(), monoLeft->getResolutionHeight());

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    edgeDetectorRgb->setMaxOutputFrameSize(8294400);

    sysLog->setRate(0.2f);

#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
    std::string source1 = R"(
        import time
        import json

        counter = 0
        while True:
            b = Buffer(64)
            dict = {'counter1': counter}
            b.setData(json.dumps(dict).encode('utf-8'))
            node.io['out'].send(b)
            time.sleep(0.01)
            counter = counter + 1
    )";
    std::string source2 = R"(
        import json
        import time

        counter = 0
        MIN_BUFFER_SIZE = 128
        while True:
            t = int( time.time() * 1000.0 )
            x = ( ((t & 0xff000000) >> 24) + ((t & 0x00ff0000) >> 8) + ((t & 0x0000ff00) << 8) + ((t & 0x000000ff) << 24))
            rand = x % 500

            buffers = node.io['in'].tryGetAll()

            dicts = []
            for b in buffers:
                data = node.io['in'].get().getData()
                jsonStr = str(data, 'utf-8')
                dict = json.loads(jsonStr)
                dict['counter2'] = counter
                dicts.append(dict)

            dicts.append({'counter1': None, 'counter2': counter})

            for dict in dicts:
                b = Buffer(MIN_BUFFER_SIZE + rand * 16)
                b.setData(json.dumps(dict).encode('utf-8'))
                node.io['out'].send(b)

            time.sleep(rand / 1000)
            counter = counter + 1
    )";

    script1->setScript(source1);
    script2->setScript(source2);
    script3->setScript(source1);
    script4->setScript(source2);
    script3->setProcessor(dai::ProcessorType::LEON_MSS);
    script4->setProcessor(dai::ProcessorType::LEON_MSS);

    scriptBurn->setScript(R"(
        import json
        import time
        counter = 0
        while True:
            rand = 0
            for x in range(0, 10):
                t = int( time.time() * 1000.0 )
                x = ( ((t & 0xff000000) >> 24) + ((t & 0x00ff0000) >> 8) + ((t & 0x0000ff00) << 8) + ((t & 0x000000ff) << 24))
                rand = x % 50
            counter = counter + rand
            time.sleep(0.001)
    )");
    scriptBurn->setProcessor(dai::ProcessorType::LEON_MSS);
#endif

    // // By default the least mount of resources are allocated
    // // increasing it improves performance when optical flow is enabled
    // auto numShaves = 2;
    // auto numMemorySlices = 2;
    // featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    // featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    // Linking
    monoLeft->out.link(ve1->input);
    camRgb->video.link(ve2->input);
    monoRight->out.link(ve3->input);

    ve1->bitstream.link(ve1Out->input);
    ve2->bitstream.link(ve2Out->input);
    ve3->bitstream.link(ve3Out->input);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    camRgb->preview.link(spatialDetectionNetwork->input);
    camRgb->preview.link(xoutRgb->input);
    spatialDetectionNetwork->out.link(xoutNN->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    monoLeft->out.link(edgeDetectorLeft->inputImage);
    monoRight->out.link(edgeDetectorRight->inputImage);
    camRgb->video.link(edgeDetectorRgb->inputImage);
    edgeDetectorLeft->outputImage.link(xoutEdgeLeft->input);
    edgeDetectorRight->outputImage.link(xoutEdgeRight->input);
    sysLog->out.link(xoutSysLog->input);
    xoutSysLog->input.setBlocking(false);
    xoutSysLog->input.setQueueSize(1);

#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
    script1->outputs["out"].link(script2->inputs["in"]);
    script2->outputs["out"].link(scriptOut->input);
    script3->outputs["out"].link(script4->inputs["in"]);
    script4->outputs["out"].link(scriptOut2->input);
#endif

    // monoLeft->out.link(featureTrackerLeft->inputImage);
    // featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);
    // monoRight->out.link(featureTrackerRight->inputImage);
    // featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);

    // Do not send out rgb edge
    // edgeDetectorRgb->outputImage.link(xoutEdgeRgb->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto usb_speed = device.getUsbSpeed();

    // Output queues will be used to get the encoded data from the output defined above
    auto outQ1 = device.getOutputQueue("ve1Out", 30, false);
    auto outQ2 = device.getOutputQueue("ve2Out", 30, false);
    auto outQ3 = device.getOutputQueue("ve3Out", 30, false);
    auto previewQueue = device.getOutputQueue("rgb", 4, false);
    auto depthQueue = device.getOutputQueue("depth", 4, false);
    auto detectionNNQueue = device.getOutputQueue("detections", 4, false);
    auto edgeLeftQueue = device.getOutputQueue(edgeLeftStr, 8, false);
    auto edgeRightQueue = device.getOutputQueue(edgeRightStr, 8, false);
    auto edgeRgbQueue = device.getOutputQueue(edgeRgbStr, 8, false);
    auto qSysInfo = device.getOutputQueue("sysinfo", 4, false);
#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
    auto scriptQueue = device.getOutputQueue("script", 8, false);
    auto script2Queue = device.getOutputQueue("script2", 8, false);
#endif

    // auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    // auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);

#ifdef DEPTHAI_STABILITY_TEST_DEBUG
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);
#endif

    mutex countersMtx;
    unordered_map<std::string, int> counters;

    thread countingThread([&countersMtx, &counters, &device, &usb_speed, TEST_TIMEOUT]() {
        // Initial delay
        this_thread::sleep_for(5s);

        auto timeoutStopwatch = steady_clock::now();
        auto fiveFpsCounter = steady_clock::now();
        while(alive) {
            if(steady_clock::now() - fiveFpsCounter >= 5s) {
                unique_lock<mutex> l(countersMtx);

                bool failed = counters.size() == 0;
                cout << "[" << duration_cast<seconds>(steady_clock::now() - timeoutStopwatch).count() << "s] "
                     << "Usb speed " << usb_speed << " "
                     << "FPS: ";
                for(const auto& kv : counters) {
                    if(kv.second == 0) {
                        failed = true;
                    }

                    cout << kv.first << ": " << kv.second / 5.0f << ", ";
                }
                cout << "\n";

                if(failed) {
                    cout << "Didn't recieve enough frames in time...\n";
                    exit(1);
                }

                counters = {};
                fiveFpsCounter = steady_clock::now();
            }

            if(steady_clock::now() - timeoutStopwatch > TEST_TIMEOUT) {
                alive = false;
                break;
            }

            this_thread::sleep_for(500ms);
        }

        // give 5s for graceful shutdown
        this_thread::sleep_for(5s);
        device.close();
    });

    while(alive) {
        auto out1 = outQ1->tryGetAll<dai::ImgFrame>();
        auto out2 = outQ2->tryGetAll<dai::ImgFrame>();
        auto out3 = outQ3->tryGetAll<dai::ImgFrame>();
        auto imgFrame = previewQueue->get<dai::ImgFrame>();
        auto inDet = detectionNNQueue->get<dai::SpatialImgDetections>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        auto edgeLefts = edgeLeftQueue->tryGetAll<dai::ImgFrame>();
        auto edgeRights = edgeRightQueue->tryGetAll<dai::ImgFrame>();

        auto sysInfo = qSysInfo->tryGet<dai::SystemInformation>();
        if(sysInfo) {
            printf("----------------------------------------\n");
            std::cout << "Usb speed: " << usb_speed << std::endl;
            printSystemInformation(*sysInfo);
            printf("----------------------------------------\n");
        }
#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
        auto script = scriptQueue->tryGetAll<dai::Buffer>();
        auto script2 = script2Queue->tryGetAll<dai::Buffer>();
#endif

        // auto edgeRgbs = edgeRgbQueue->getAll<dai::ImgFrame>();

        // auto trackedLefts = outputFeaturesLeftQueue->getAll<dai::TrackedFeatures>();
        // auto trackedRigths = outputFeaturesRightQueue->getAll<dai::TrackedFeatures>();

        {
            unique_lock<mutex> l(countersMtx);
            counters["out1"] += out1.size();
            counters["out2"] += out2.size();
            counters["out3"] += out3.size();
            counters["imgFrame"]++;
            counters["inDet"]++;
            counters["depth"]++;
            counters["edgeLefts"] += edgeLefts.size();
            counters["edgeRights"] += edgeRights.size();

#ifdef DEPTHAI_STABILITY_TEST_SCRIPT
            counters["script"] += script.size();
            counters["script2"] += script2.size();
#endif
            // counters["trackedLefts"] += trackedLefts.size();
            // counters["trackedRights"] += trackedRigths.size();
        }

#ifdef DEPTHAI_STABILITY_TEST_DEBUG

        /// DISPLAY & OPENCV Section
        for(const auto& edgeLeft : edgeLefts) {
            cv::Mat edgeLeftFrame = edgeLeft->getFrame();
            cv::imshow(edgeLeftStr, edgeLeftFrame);
        }
        for(const auto& edgeRight : edgeRights) {
            cv::Mat edgeRightFrame = edgeRight->getFrame();
            cv::imshow(edgeRightStr, edgeRightFrame);
        }
        // for(const auto& edgeRgb : edgeRgbs) {
        //     cv::Mat edgeRgbFrame = edgeRgb->getFrame();
        //     cv::imshow(edgeRgbStr, edgeRgbFrame);
        // }

        cv::Mat frame = imgFrame->getCvFrame();
        cv::Mat depthFrame = depth->getFrame();  // depthFrame values are in millimeters

        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto detections = inDet->detections;

        for(const auto& detection : detections) {
            auto roiData = detection.boundingBoxMapping;
            auto roi = roiData.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto topLeft = roi.topLeft();
            auto bottomRight = roi.bottomRight();
            auto xmin = (int)topLeft.x;
            auto ymin = (int)topLeft.y;
            auto xmax = (int)bottomRight.x;
            auto ymax = (int)bottomRight.y;
            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);

            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream depthX;
            depthX << "X: " << (int)detection.spatialCoordinates.x << " mm";
            cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthY;
            depthY << "Y: " << (int)detection.spatialCoordinates.y << " mm";
            cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)detection.spatialCoordinates.z << " mm";
            cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("rgb", frame);

    #ifdef DEPTHAI_STABILITY_TEST_SCRIPT
        for(auto json : script) {
            if(json != nullptr) {
                std::cout << "Script: " << std::string((const char*)json->getData().data(), json->getData().size()) << std::endl;
            }
        }
        for(auto json : script2) {
            if(json != nullptr) {
                std::cout << "Script2: " << std::string((const char*)json->getData().data(), json->getData().size()) << std::endl;
            }
        }
    #endif

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            alive = false;
            break;
        }

#endif
    }

    // Clean up the counting thread
    alive = false;
    if(countingThread.joinable()) countingThread.join();

    return 0;
}
