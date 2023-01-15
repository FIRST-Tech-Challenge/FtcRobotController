package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "JunctionDetectionTest", group = "Autonomous")
public class AutonomousJunctionDetection extends BaseAutonomous {

    OpenCvCamera webcam;
    ColorDetectPipeline pipeline = new JunctionDetectPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        initialize();
        waitForStart();

        while (opModeIsActive()) {
            if(pipeline.rectDetected)
                driveWithIMU(pipeline.detectedRect.x/800.0, 0, 0);

            telemetry.addData("x", pipeline.detectedRect.x);
            telemetry.addData("y", pipeline.detectedRect.y);
            telemetry.addData("🏃‍💨", pipeline.isRunning);
            telemetry.addData("counter", pipeline.counter);
            telemetry.addData("🟨", pipeline.rectDetected);
            telemetry.addData("contour area", pipeline.biggestContourArea);
            telemetry.addData("bounding box area", pipeline.biggestContourBoundingBoxArea);
            telemetry.update();
        }
    }
}
