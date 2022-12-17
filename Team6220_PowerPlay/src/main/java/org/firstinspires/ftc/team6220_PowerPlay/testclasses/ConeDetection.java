package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class ConeDetection extends AprilTagDetect {
    public ConeDetectionPipeline coneDetectionPipeline;
    OpenCvCamera camera;

    public void detectGrab() {
        // initializes camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        coneDetectionPipeline = new ConeDetectionPipeline();

        camera.setPipeline(coneDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}

