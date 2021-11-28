package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VISION_DATA.CAMERA_HEIGHT;
import static org.firstinspires.ftc.teamcode.vision.VISION_DATA.CAMERA_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class TSEDetectorCam {
    OpenCvCamera phoneCam;
    TSEDetectorPipeline pipeline;

    public TSEDetectorCam(Telemetry telemetry) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        pipeline = new TSEDetectorPipeline(telemetry);
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });
    }

    public TSEDetectorCam(Telemetry telemetry, FtcDashboard dashboard) {
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        dashboard.startCameraStream(phoneCam, 10);

        pipeline = new TSEDetectorPipeline(telemetry);
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });
    }

    public TSE_POSITION detectElement() {
        return pipeline.getPos();
    }

    public void close() {
        phoneCam.stopStreaming();
        phoneCam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() { }
        });
    }
}
