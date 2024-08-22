#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setProcessor(dai::ProcessorType::LEON_CSS);
    script->setScript(R"(
        import time

        cal = Device.readCalibration2()
        left_camera_id = cal.getStereoLeftCameraId()
        right_camera_id = cal.getStereoRightCameraId()

        extrinsics = cal.getCameraExtrinsics(left_camera_id, right_camera_id)
        intrinsics_left = cal.getCameraIntrinsics(left_camera_id)

        node.info(extrinsics.__str__())
        node.info(intrinsics_left.__str__())

        time.sleep(1)
        node.io['end'].send(Buffer(32))
    )");

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("end");

    script->outputs["end"].link(xout->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);

    device.getOutputQueue("end")->get();

    return 0;
}
