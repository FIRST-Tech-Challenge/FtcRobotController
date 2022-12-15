package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptCompassCalibration;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "JunctionDetectionTest", group = "Autonomous")
public class AutonomousJunctionDetection extends BaseAutonomous {

    OpenCvCamera webcam;
    ColorDetectPipeline pipeline = new JunctionDetectPipeline();

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize();
        telemetry.addLine("opening camera...");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Dino"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("camera is open");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        waitForStart();
        while (opModeIsActive()) {

            if(pipeline.rectDetected) {
                driveWithIMU(pipeline.detectedRect.x/1280, pipeline.detectedRect.y/720, 0);
            }

            telemetry.addData("x", pipeline.detectedRect.x);
            telemetry.addData("y", pipeline.detectedRect.y);
            telemetry.addData("🏃‍💨", pipeline.isRunning);
            telemetry.addData("counter", pipeline.counter);
            telemetry.update();
        }

    }
}
