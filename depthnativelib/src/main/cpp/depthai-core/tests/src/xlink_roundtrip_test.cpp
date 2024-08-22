#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;

auto TIMEOUT = 5s;

static void test_xlink_roundtrip(int w, int h) {
    std::vector<std::uint8_t> data(w * h * 3);

    dai::Pipeline p;
    auto x_in = p.create<dai::node::XLinkIn>();
    x_in->setStreamName("to_device");
    x_in->setMaxDataSize(data.size());
    auto x_out = p.create<dai::node::XLinkOut>();
    x_out->setStreamName("to_host");
    x_in->out.link(x_out->input);

    dai::Device device(p);

    auto outQ = device.getOutputQueue("to_host");
    auto inQ = device.getInputQueue("to_device");

    dai::ImgFrame imgFrame;

    imgFrame.setSequenceNum(123);
    imgFrame.setWidth(w);
    imgFrame.setHeight(h);

    imgFrame.setData(data);
    imgFrame.setType(dai::RawImgFrame::Type::BGR888p);
    // Send the frame
    inQ->send(imgFrame);

    // TODO: add timeout
    auto t1 = steady_clock::now();
    bool success = false;
    do {
        auto retFrm = outQ->tryGet();
        if(retFrm) {
            REQUIRE(imgFrame.getSequenceNum() == 123);
            return;
        }
    } while(steady_clock::now() - t1 < TIMEOUT);
    // Timeout
    FAIL("Timeout receiving back the sent message");
}

TEST_CASE("Test XLinkIn->XLinkOut passthrough with random 1000x1000 frame") {
    test_xlink_roundtrip(1000, 1000);
}

TEST_CASE("Test XLinkIn->XLinkOut passthrough with random 2000x1000 frame") {
    test_xlink_roundtrip(2000, 1000);
}

TEST_CASE("Test XLinkIn->XLinkOut passthrough with random 4000x3000 frame") {
    test_xlink_roundtrip(4000, 3000);
}