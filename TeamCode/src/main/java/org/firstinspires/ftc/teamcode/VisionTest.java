package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class VisionTest extends LinearOpMode {
    OpenCvWebcam webcam;
    SignalPipeline pipeLine;
    
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        pipeLine = new SignalPipeline();
        webcam.setPipeline(pipeLine);
        
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        telemetry.addLine("Working");
        telemetry.update();
        
        while (opModeIsActive()) {
            telemetry.addData("Mean Hue", pipeLine.meanHue);
            telemetry.addData("Corners", pipeLine.foundCorners);
            telemetry.addData("State by Hue", pipeLine.stateByHue);
            telemetry.addData("State by Corners", pipeLine.stateByCorners);
            telemetry.update();
            
            sleep(50);
        }
    }
}
