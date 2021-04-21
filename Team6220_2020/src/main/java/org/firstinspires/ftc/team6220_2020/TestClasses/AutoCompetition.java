package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_2020.MasterAutonomous;
import org.firstinspires.ftc.team6220_2020.RingDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Autonomous", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        Initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new RingDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        int ringStackHeight = RingDetectionPipeline.ringStackHeight;

        pauseMillis(100);

        if (RingDetectionPipeline.ringStackHeight != ringStackHeight) {
            ringStackHeight = RingDetectionPipeline.ringStackHeight;
        }

        telemetry.addData("Num rings: ", ringStackHeight);
        telemetry.update();

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        switch (ringStackHeight) {
            case 0:
                driveInches(72, 90, 0.5);
                pauseMillis(250);
                driveInches(40, 0, 1.0);
                driveLauncher(0.9);
                pauseMillis(500);
                fireLauncher();
                driveInches(7.5, 0, 1.0);
                pauseMillis(500);
                fireLauncher();
                driveInches(7.5, 0, 1.0);
                pauseMillis(500);
                fireLauncher();
                driveLauncher(0.0);
                driveInches(6, 90, 1.0);
                break;

            case 1:
                driveInches(72, 90, 0.5);
                pauseMillis(250);
                driveInches((24 * Math.sqrt(2.0)), 45, 0.5);
                pauseMillis(250);
                driveInches(24, 270, 1.0);
                pauseMillis(250);
                driveLauncher(0.9);
                pauseMillis(500);
                fireLauncher();
                driveLauncher(0.0);
                driveInches(12, 270, 1.0);
                driveZiptie(1.0);
                driveBelt(1.0);
                pauseMillis(1000);
                driveZiptie(0.0);
                driveBelt(0.0);
                driveInches(12, 90, 1.0);
                driveInches(16, 0, 1.0);
                driveLauncher(0.9);
                pauseMillis(500);
                fireLauncher();
                driveInches(7.5, 0, 1.0);
                pauseMillis(500);
                fireLauncher();
                driveInches(7.5, 0, 1.0);
                pauseMillis(500);
                fireLauncher();
                driveLauncher(0.0);
                driveInches(6, 90, 1.0);
                break;

            case 4:
                driveInches(120, 90,  0.5);
                pauseMillis(250);
                driveInches(48, 270, 1.0);
                pauseMillis(250);
                driveInches(40, 0, 1.0);
                driveLauncher(0.9);
                pauseMillis(500);
                fireLauncher();
                driveInches(7.5, 0, 1.0);
                pauseMillis(500);
                fireLauncher();
                driveInches(7.5, 0, 1.0);
                pauseMillis(500);
                fireLauncher();
                driveLauncher(0.0);
                driveInches(6, 90, 1.0);
                break;
        }
    }
}
