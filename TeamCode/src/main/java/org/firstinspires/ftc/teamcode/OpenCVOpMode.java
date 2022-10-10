package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="Skystone Detector", group="Auto")
public class OpenCVOpMode extends LinearOpMode {
    OpenCvCamera cam;
//    OpenCVTest pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        OpenCVTest detector = new OpenCVTest(telemetry);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        cam.setPipeline(detector);
        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case RIGHT:
                // ...
                break;
            case NOT_FOUND:
                // ...
        }
        cam.stopStreaming();
    }
}