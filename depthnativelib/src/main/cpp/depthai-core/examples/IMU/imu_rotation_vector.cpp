
#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("imu");

    // enable ROTATION_VECTOR at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 400);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);

    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOut->input);

    dai::Device d(pipeline);

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    auto baseTs = steady_clock::now();

    while(true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            auto& rVvalues = imuPacket.rotationVector;

            auto rvTs = rVvalues.getTimestampDevice() - baseTs;
            printf("Rotation vector timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(rvTs).count()));

            printf(
                "Quaternion: i: %.3f j: %.3f k: %.3f real: %.3f\n"
                "Accuracy (rad): %.3f \n",
                rVvalues.i,
                rVvalues.j,
                rVvalues.k,
                rVvalues.real,
                rVvalues.rotationVectorAccuracy);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }

    return 0;
}
