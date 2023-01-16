package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class GrabberCamera extends BaseAutonomous {
    public RobotCameraPipeline GrabberCameraPipeline;
    OpenCvCamera camera;

    public void detectGrabGrabberCamera(int[] lower, int[] upper) {
        // initializes camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "GrabberCamera"), cameraMonitorViewId);
        //Create new instance of ConeDetectionPipeline
        GrabberCameraPipeline = new RobotCameraPipeline();
        //Set ranges for the pipeline
        GrabberCameraPipeline.setRanges(lower, upper);
        //Assign pipeline to the camera
        camera.setPipeline(GrabberCameraPipeline);
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

