package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutonomousCompetition extends MasterAutonomous {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        Initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new RingDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        int ringStackHeight = RingDetectionPipeline.ringStackHeight;
        pauseMillis(100);
        if(RingDetectionPipeline.ringStackHeight != ringStackHeight){
            ringStackHeight = RingDetectionPipeline.ringStackHeight;
        }
        telemetry.addData("Num rings : ", ringStackHeight);
        telemetry.update();

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        driveInches(24 * Math.sqrt(2.0), 45, 1.0);

        switch (ringStackHeight){
            case 0:
                driveInches(50,90, 1.0);
                pauseMillis(200);
                driveInches(6,-90, 1.0);
                break;

            case 1:

            case 4:
                driveInches(52,90, 1.0);
                pauseMillis(200);
                driveInches(10,-90, 1.0);
                break;
        }

        driveInches(4,180, 1.0);

        driveLauncher(0.94);

        pauseMillis(1500);

        for(int i = 0; i < 3; i++){
            fireLauncher(0);
            driveInches(7.5,0, 0.2);
            pauseMillis(750);
        }

        driveLauncher(0.0);

        switch (ringStackHeight){
            case 0:
                driveInches(16,90, 1.0);

                //turnToAngle(45);
                turnToAngle(90);

                //New method
                driveInches(54,90, 0.5);

                driveInches(24,-90, 1.0);
                break;

            case 1:

                turnToAngle(43);

                //New method
                driveInches(42,90, 0.5);

                driveInches(30,-90, 1.0);
                break;

            case 4:

                turnToAngle(43);

                //New method
                driveInches(80,90, 0.5);

                driveInches(64,-90, 1.0);
                break;
        }

    }
}