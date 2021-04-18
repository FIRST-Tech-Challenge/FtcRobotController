package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_2020.TestClasses.OpenCVTest;
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


        //for(int i = 0; i < 100; i++){
        //    telemetry.addData("pixel: ", RingDetectionPipeline.ringPixelArea);
        //    telemetry.update();
        //    sleep(100);
        //}


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

        driveInches(12 * Math.sqrt(2.0), 45);

        switch (ringStackHeight){
            case 0:
                driveInches(50,90);
                pauseMillis(200);
                driveInches(6,-90);
                break;

            case 1:
                driveInches(74,90);
                pauseMillis(200);
                driveInches(30,-90);
                break;

            case 4:
                driveInches(106,90);
                pauseMillis(200);
                driveInches(54,-90);
                break;
        }

        //todo align to left wobble goal

        driveInches(7.5,0);

        for(int i = 0; i < 3; i++){
            fireLauncher(1350);
            driveInches(7.5,0);
        }

        driveLauncher(0.0);

        switch (ringStackHeight){
            case 0:
                driveInches(6,90);
                turnToAngle(180);
                driveInches(64,-90);
                driveInches(24,90);
                break;

            case 1:
                driveInches(30,90);
                turnToAngle(180);
                driveInches(40,-90);
                driveInches(24,90);
                driveInches(24,180);
                break;

            case 4:
                driveInches(60,90);
                turnToAngle(-179);
                driveInches(64,-90);
                driveInches(24,90);
                driveInches(48,180);
                break;
        }

    }
}