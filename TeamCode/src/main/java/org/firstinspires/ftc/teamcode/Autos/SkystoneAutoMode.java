package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Pipelines.SkystoneDetectorPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="SkystoneDetectorPipeline", group="Auto")
public class SkystoneAutoMode extends LinearOpMode {
    OpenCvCamera webCam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                 "id", hardwareMap.appContext.getPackageName());

        webCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        SkystoneDetectorPipeline detector = new SkystoneDetectorPipeline(telemetry);

        webCam.setPipeline(detector);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                telemetry.update();
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //This will be called if the camera could not be opened
            }
        });

        waitForStart();

        switch (detector.getSkystoneLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        webCam.closeCameraDevice();
        // webCam.stopStreaming();
    }
}