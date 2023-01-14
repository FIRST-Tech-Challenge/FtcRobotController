package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class ConeDetection extends BaseAutonomous {
    public ConeDetectionPipeline coneDetectionPipeline;
    OpenCvCamera camera;

    public void detectGrab(int[] lower, int[] upper) {
        // initializes camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        //Create new instance of ConeDetectionPipeline
        coneDetectionPipeline = new ConeDetectionPipeline();
        //Set ranges for the pipeline
        coneDetectionPipeline.setRanges(lower, upper);
        //Assign pipeline to the camera
        camera.setPipeline(coneDetectionPipeline);
        //Starts streaming the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }
}

