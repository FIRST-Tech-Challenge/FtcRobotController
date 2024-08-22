#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr auto FPS = 30;

int main() {
    dai::Pipeline pipeline;

    // Define a source - color camera
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    // Since we are saving RGB frames in Script node we need to make the
    // video pool size larger, otherwise the pipeline will freeze because
    // the ColorCamera won't be able to produce new video frames.
    camRgb->setVideoNumFramesPool(10);
    camRgb->setFps(FPS);

    auto left = pipeline.create<dai::node::MonoCamera>();
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setCamera("left");
    left->setFps(FPS);

    auto right = pipeline.create<dai::node::MonoCamera>();
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setCamera("right");
    right->setFps(FPS);

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(false);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    left->out.link(stereo->left);
    right->out.link(stereo->right);

    // Script node will sync high-res frames
    auto script = pipeline.create<dai::node::Script>();

    // Send both streams to the Script node so we can sync them
    stereo->disparity.link(script->inputs["disp_in"]);
    camRgb->video.link(script->inputs["rgb_in"]);

    script->setScript(R"(
        FPS=30
        import time
        from datetime import timedelta
        import math

        # Timestamp threshold (in miliseconds) under which frames will be considered synced.
        # Lower number means frames will have less delay between them, which can potentially
        # lead to dropped frames.
        MS_THRESHOL=math.ceil(500 / FPS)

        def check_sync(queues, timestamp):
            matching_frames = []
            for name, list in queues.items(): # Go through each available stream
                # node.warn(f"List {name}, len {str(len(list))}")
                for i, msg in enumerate(list): # Go through each frame of this stream
                    time_diff = abs(msg.getTimestamp() - timestamp)
                    if time_diff <= timedelta(milliseconds=MS_THRESHOL): # If time diff is below threshold, this frame is considered in-sync
                        matching_frames.append(i) # Append the position of the synced frame, so we can later remove all older frames
                        break

            if len(matching_frames) == len(queues):
                # We have all frames synced. Remove the excess ones
                i = 0
                for name, list in queues.items():
                    queues[name] = queues[name][matching_frames[i]:] # Remove older (excess) frames
                    i+=1
                return True
            else:
                return False # We don't have synced frames yet

        names = ['disp', 'rgb']
        frames = dict() # Dict where we store all received frames
        for name in names:
            frames[name] = []

        while True:
            for name in names:
                f = node.io[name+"_in"].tryGet()
                if f is not None:
                    frames[name].append(f) # Save received frame

                    if check_sync(frames, f.getTimestamp()): # Check if we have any synced frames
                        # Frames synced!
                        node.info(f"Synced frame!")
                        for name, list in frames.items():
                            syncedF = list.pop(0) # We have removed older (excess) frames, so at positions 0 in dict we have synced frames
                            node.info(f"{name}, ts: {str(syncedF.getTimestamp())}, seq {str(syncedF.getSequenceNum())}")
                            node.io[name+'_out'].send(syncedF) # Send synced frames to the host


            time.sleep(0.001)  # Avoid lazy looping
        )");

    std::vector<std::string> scriptOut{"disp", "rgb"};
    // Create XLinkOut for disp/rgb streams
    for(auto& name : scriptOut) {
        auto xout = pipeline.create<dai::node::XLinkOut>();
        xout->setStreamName(name);
        script->outputs[name + "_out"].link(xout->input);
    }

    dai::Device device(pipeline);

    device.setLogOutputLevel(dai::LogLevel::INFO);
    device.setLogLevel(dai::LogLevel::INFO);
    std::vector<std::string> names{"rgb", "disp"};
    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> streams;
    for(auto& name : names) {
        streams[name] = device.getOutputQueue(name);
    }
    while(true) {
        for(auto& iter : streams) {
            auto name = iter.first;
            auto queue = iter.second;
            auto img = queue->get<dai::ImgFrame>();
            // Display timestamp/sequence number of two synced frames
            std::cout << "Stream " << name << ", timestamp: " << img->getTimestamp().time_since_epoch().count()
                      << ", sequence number: " << img->getSequenceNum() << std::endl;
        }
    }
}
