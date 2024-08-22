#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

// Number of IO to test
constexpr auto NUM_CONN = 16;

// Using
using namespace std;
using namespace dai;
using namespace std::chrono;
using namespace std::chrono_literals;

TEST_CASE("Test many IO connections after crossing Leon processors") {
    Pipeline pipeline;
    auto camRgb = pipeline.create<node::ColorCamera>();
    camRgb->setInterleaved(true);

    for(int i = 0; i < NUM_CONN; i++) {
        string out = "out" + to_string(i);
        auto xout = pipeline.create<node::XLinkOut>();
        xout->setStreamName(out);
        camRgb->preview.link(xout->input);
    }

    Device device(pipeline);

    this_thread::sleep_for(5s);

    // Set queues to non-blocking
    std::unordered_map<string, int> numFrames;
    for(int i = 0; i < NUM_CONN; i++) {
        string out = "out" + to_string(i);
        device.getOutputQueue(out, 9, false);
        numFrames[out] = 0;
    }

    // Retrieve at least 10 frames of each, without errors
    constexpr auto NUM_FRAMES_REQUIRED = 10;

    bool receivedEnoughFrames = true;
    auto t1 = steady_clock::now();
    do {
        receivedEnoughFrames = true;
        // Set queues to non-blocking
        for(int i = 0; i < NUM_CONN; i++) {
            string out = "out" + to_string(i);

            auto frame = device.getOutputQueue(out, 9, false)->tryGet();
            if(frame != nullptr) {
                numFrames[out]++;
            }

            // Check
            if(numFrames[out] < NUM_FRAMES_REQUIRED) {
                receivedEnoughFrames = false;
            }
        }

        if(receivedEnoughFrames) {
            break;
        }
    } while(steady_clock::now() - t1 < 5s);

    cout << "numFrames: ";
    for(int i = 0; i < NUM_CONN; i++) {
        string out = "out" + to_string(i);
        cout << numFrames[out] << ", ";
    }
    cout << "\n";

    REQUIRE(receivedEnoughFrames == true);
}

TEST_CASE("Test many IO connections before crossing Leon processors") {
    Pipeline pipeline;
    auto camRgb = pipeline.create<node::ColorCamera>();
    camRgb->setInterleaved(false);

    for(int i = 0; i < NUM_CONN; i++) {
        string out = "out" + to_string(i);
        auto manip = pipeline.create<node::ImageManip>();
        camRgb->preview.link(manip->inputImage);
        auto xout = pipeline.create<node::XLinkOut>();
        xout->setStreamName(out);
        manip->out.link(xout->input);
    }

    Device device(pipeline);

    // Set queues to non-blocking
    std::unordered_map<string, int> numFrames;
    for(int i = 0; i < NUM_CONN; i++) {
        string out = "out" + to_string(i);
        device.getOutputQueue(out, 9, false);
        numFrames[out] = 0;
    }

    // Retrieve at least 10 frames of each, without errors
    constexpr auto NUM_FRAMES_REQUIRED = 10;

    bool receivedEnoughFrames = false;
    auto t1 = steady_clock::now();
    do {
        receivedEnoughFrames = true;
        // Set queues to non-blocking
        for(int i = 0; i < NUM_CONN; i++) {
            string out = "out" + to_string(i);
            auto frame = device.getOutputQueue(out, 9, false)->tryGet();
            if(frame != nullptr) {
                numFrames[out]++;
            }

            // Check
            if(numFrames[out] < NUM_FRAMES_REQUIRED) {
                receivedEnoughFrames = false;
            }
        }

        if(receivedEnoughFrames) {
            break;
        }

    } while(steady_clock::now() - t1 < 5s);

    cout << "numFrames: ";
    for(int i = 0; i < NUM_CONN; i++) {
        string out = "out" + to_string(i);
        cout << numFrames[out] << ", ";
    }
    cout << "\n";

    REQUIRE(receivedEnoughFrames == true);
}
