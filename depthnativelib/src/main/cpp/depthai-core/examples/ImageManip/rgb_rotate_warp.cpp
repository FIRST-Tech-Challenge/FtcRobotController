#include <iostream>

#include "depthai/depthai.hpp"
#include "utility.hpp"

static constexpr auto keyRotateDecr = 'z';
static constexpr auto keyRotateIncr = 'x';
static constexpr auto keyResizeInc = 'v';
static constexpr auto keyWarpTestCycle = 'c';

void printControls() {
    printf("\n=== Controls:\n");
    printf(" %c -rotated rectangle crop, decrease rate\n", keyRotateDecr);
    printf(" %c -rotated rectangle crop, increase rate\n", keyRotateIncr);
    printf(" %c -warp 4-point transform, cycle through modes\n", keyWarpTestCycle);
    printf(" %c -resize cropped region, or disable resize\n", keyResizeInc);
    printf(" h -print controls (help)\n");
}

static constexpr auto ROTATE_RATE_MAX = 5.0f;
static constexpr auto ROTATE_RATE_INC = 0.1f;

static constexpr auto RESIZE_MAX_W = 800;
static constexpr auto RESIZE_MAX_H = 600;
static constexpr auto RESIZE_FACTOR_MAX = 5;

/* The crop points are specified in clockwise order,
 * with first point mapped to output top-left, as:
 *   P0  ->  P1
 *    ^       v
 *   P3  <-  P2
 */
static const dai::Point2f P0 = {0, 0};  // top-left
static const dai::Point2f P1 = {1, 0};  // top-right
static const dai::Point2f P2 = {1, 1};  // bottom-right
static const dai::Point2f P3 = {0, 1};  // bottom-left
struct warpFourPointTest {
    std::vector<dai::Point2f> points;
    bool normalizedCoords;
    const char* description;
};

std::vector<warpFourPointTest> warpList = {
    //{{{  0,  0},{  1,  0},{  1,  1},{  0,  1}}, true, "passthrough"},
    //{{{  0,  0},{639,  0},{639,479},{  0,479}}, false,"passthrough (pixels)"},
    {{P0, P1, P2, P3}, true, "1. passthrough"},
    {{P3, P0, P1, P2}, true, "2. rotate 90"},
    {{P2, P3, P0, P1}, true, "3. rotate 180"},
    {{P1, P2, P3, P0}, true, "4. rotate 270"},
    {{P1, P0, P3, P2}, true, "5. horizontal mirror"},
    {{P3, P2, P1, P0}, true, "6. vertical flip"},
    {{{-0.1f, -0.1f}, {1.1f, -0.1f}, {1.1f, 1.1f}, {-0.1f, 1.1f}}, true, "7. add black borders"},
    {{{-0.3f, 0}, {1, 0}, {1.3f, 1}, {0, 1}}, true, "8. parallelogram transform"},
    {{{-0.2f, 0}, {1.8f, 0}, {1, 1}, {0, 1}}, true, "9. trapezoid transform"},
};

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto manip = pipeline.create<dai::node::ImageManip>();

    auto camOut = pipeline.create<dai::node::XLinkOut>();
    auto manipOut = pipeline.create<dai::node::XLinkOut>();
    auto manipCfg = pipeline.create<dai::node::XLinkIn>();

    camOut->setStreamName("preview");
    manipOut->setStreamName("manip");
    manipCfg->setStreamName("manipCfg");

    // Properties
    camRgb->setPreviewSize(640, 480);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    manip->setMaxOutputFrameSize(2000 * 1500 * 3);

    // Linking
    camRgb->preview.link(camOut->input);
    camRgb->preview.link(manip->inputImage);
    manip->out.link(manipOut->input);
    manipCfg->out.link(manip->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Create input & output queues
    auto qPreview = device.getOutputQueue("preview", 8, false);
    auto qManip = device.getOutputQueue("manip", 8, false);
    auto qManipCfg = device.getInputQueue("manipCfg");

    std::vector<decltype(qPreview)> frameQueues{qPreview, qManip};

    // keep processing data
    int key = -1;
    float angleDeg = 0;
    float rotateRate = 1.0;
    int resizeFactor = 0;
    int resizeX = 0;
    int resizeY = 0;
    bool testFourPt = false;
    int warpIdx = -1;

    printControls();

    while(key != 'q') {
        if(key >= 0) {
            printf("Pressed: %c | ", key);
            if(key == keyRotateDecr || key == keyRotateIncr) {
                if(key == keyRotateDecr) {
                    if(rotateRate > -ROTATE_RATE_MAX) rotateRate -= ROTATE_RATE_INC;
                } else if(key == keyRotateIncr) {
                    if(rotateRate < ROTATE_RATE_MAX) rotateRate += ROTATE_RATE_INC;
                }
                testFourPt = false;
                printf("Crop rotated rectangle, rate: %g degrees", rotateRate);
            } else if(key == keyResizeInc) {
                resizeFactor++;
                if(resizeFactor > RESIZE_FACTOR_MAX) {
                    resizeFactor = 0;
                    printf("Crop region not resized");
                } else {
                    resizeX = RESIZE_MAX_W / resizeFactor;
                    resizeY = RESIZE_MAX_H / resizeFactor;
                    printf("Crop region resized to: %d x %d", resizeX, resizeY);
                }
            } else if(key == keyWarpTestCycle) {
                resizeFactor = 0;  // Disable resizing initially
                warpIdx = (warpIdx + 1) % warpList.size();
                printf("Warp 4-point transform: %s", warpList[warpIdx].description);
                testFourPt = true;
            } else if(key == 'h') {
                printControls();
            }
            printf("\n");
        }

        // Send an updated config with continuous rotate, or after a key press
        if(key >= 0 || (!testFourPt && std::abs(rotateRate) > 0.0001)) {
            dai::ImageManipConfig cfg;
            if(testFourPt) {
                cfg.setWarpTransformFourPoints(warpList[warpIdx].points, warpList[warpIdx].normalizedCoords);
            } else {
                angleDeg += rotateRate;
                dai::RotatedRect rr = {{320, 240},  // center
                                       {640, 480},  //{400, 400}, // size
                                       angleDeg};
                cfg.setCropRotatedRect(rr, false);
            }
            if(resizeFactor > 0) {
                cfg.setResize(resizeX, resizeY);
            }
            // cfg.setWarpBorderFillColor(255, 0, 0);
            // cfg.setWarpBorderReplicatePixels();
            qManipCfg->send(cfg);
        }

        for(const auto& q : frameQueues) {
            auto img = q->get<dai::ImgFrame>();
            auto mat = toMat(img->getData(), img->getWidth(), img->getHeight(), 3, 1);
            cv::imshow(q->getName(), mat);
        }
        key = cv::waitKey(1);
    }
    return 0;
}
