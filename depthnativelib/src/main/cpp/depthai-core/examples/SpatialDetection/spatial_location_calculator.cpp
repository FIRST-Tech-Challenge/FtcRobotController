#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr float stepSize = 0.05f;

static std::atomic<bool> newConfig{false};

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialDataCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();

    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutSpatialData = pipeline.create<dai::node::XLinkOut>();
    auto xinSpatialCalcConfig = pipeline.create<dai::node::XLinkIn>();

    xoutDepth->setStreamName("depth");
    xoutSpatialData->setStreamName("spatialData");
    xinSpatialCalcConfig->setStreamName("spatialCalcConfig");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    bool lrcheck = false;
    bool subpixel = false;

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setSubpixel(subpixel);

    // Config
    dai::Point2f topLeft(0.4f, 0.4f);
    dai::Point2f bottomRight(0.6f, 0.6f);

    dai::SpatialLocationCalculatorConfigData config;
    config.depthThresholds.lowerThreshold = 100;
    config.depthThresholds.upperThreshold = 10000;
    auto calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEDIAN;
    config.calculationAlgorithm = calculationAlgorithm;
    config.roi = dai::Rect(topLeft, bottomRight);

    spatialDataCalculator->inputConfig.setWaitForMessage(false);
    spatialDataCalculator->initialConfig.addROI(config);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    spatialDataCalculator->passthroughDepth.link(xoutDepth->input);
    stereo->depth.link(spatialDataCalculator->inputDepth);

    spatialDataCalculator->out.link(xoutSpatialData->input);
    xinSpatialCalcConfig->out.link(spatialDataCalculator->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the depth frames from the outputs defined above
    auto depthQueue = device.getOutputQueue("depth", 8, false);
    auto spatialCalcQueue = device.getOutputQueue("spatialData", 8, false);
    auto spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig");

    auto color = cv::Scalar(255, 255, 255);

    std::cout << "Use WASD keys to move ROI!" << std::endl;

    while(true) {
        auto inDepth = depthQueue->get<dai::ImgFrame>();

        cv::Mat depthFrame = inDepth->getFrame();  // depthFrame values are in millimeters
        cv::Mat depthFrameColor;

        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        auto spatialData = spatialCalcQueue->get<dai::SpatialLocationCalculatorData>()->getSpatialLocations();
        for(auto depthData : spatialData) {
            auto roi = depthData.config.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto xmin = (int)roi.topLeft().x;
            auto ymin = (int)roi.topLeft().y;
            auto xmax = (int)roi.bottomRight().x;
            auto ymax = (int)roi.bottomRight().y;

            auto depthMin = depthData.depthMin;
            auto depthMax = depthData.depthMax;

            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);
            std::stringstream depthX;
            depthX << "X: " << (int)depthData.spatialCoordinates.x << " mm";
            cv::putText(depthFrameColor, depthX.str(), cv::Point(xmin + 10, ymin + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthY;
            depthY << "Y: " << (int)depthData.spatialCoordinates.y << " mm";
            cv::putText(depthFrameColor, depthY.str(), cv::Point(xmin + 10, ymin + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)depthData.spatialCoordinates.z << " mm";
            cv::putText(depthFrameColor, depthZ.str(), cv::Point(xmin + 10, ymin + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }
        // Show the frame
        cv::imshow("depth", depthFrameColor);

        int key = cv::waitKey(1);
        switch(key) {
            case 'q':
                return 0;
            case 'w':
                if(topLeft.y - stepSize >= 0) {
                    topLeft.y -= stepSize;
                    bottomRight.y -= stepSize;
                    newConfig = true;
                }
                break;
            case 'a':
                if(topLeft.x - stepSize >= 0) {
                    topLeft.x -= stepSize;
                    bottomRight.x -= stepSize;
                    newConfig = true;
                }
                break;
            case 's':
                if(bottomRight.y + stepSize <= 1) {
                    topLeft.y += stepSize;
                    bottomRight.y += stepSize;
                    newConfig = true;
                }
                break;
            case 'd':
                if(bottomRight.x + stepSize <= 1) {
                    topLeft.x += stepSize;
                    bottomRight.x += stepSize;
                    newConfig = true;
                }
                break;
            case '1':
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEAN;
                newConfig = true;
                std::cout << "Switching calculation algorithm to MEAN!" << std::endl;
                break;
            case '2':
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
                newConfig = true;
                std::cout << "Switching calculation algorithm to MIN!" << std::endl;
                break;
            case '3':
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MAX;
                newConfig = true;
                std::cout << "Switching calculation algorithm to MAX!" << std::endl;
                break;
            case '4':
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MODE;
                newConfig = true;
                std::cout << "Switching calculation algorithm to MODE!" << std::endl;
                break;
            case '5':
                calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MEDIAN;
                newConfig = true;
                std::cout << "Switching calculation algorithm to MEDIAN!" << std::endl;
                break;
            default:
                break;
        }

        if(newConfig) {
            config.roi = dai::Rect(topLeft, bottomRight);
            config.calculationAlgorithm = calculationAlgorithm;
            dai::SpatialLocationCalculatorConfig cfg;
            cfg.addROI(config);
            spatialCalcConfigInQueue->send(cfg);
            newConfig = false;
        }
    }
    return 0;
}