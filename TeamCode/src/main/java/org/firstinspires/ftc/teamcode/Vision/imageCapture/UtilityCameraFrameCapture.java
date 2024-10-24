package org.firstinspires.ftc.teamcode.Vision.imageCapture;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class UtilityCameraFrameCapture extends LinearOpMode {
    OpenCvCamera camera;
    CaptureImagePipeline captureImagePipeline;
    boolean x;
    boolean lastX;

    public void runOpMode() {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        captureImagePipeline=new CaptureImagePipeline();
        camera.setPipeline(captureImagePipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            boolean x = gamepad1.x;
            if (x&&!lastX){

                captureImagePipeline.saveMatToDisk();
            }
            lastX=x;
        }
    }


}

