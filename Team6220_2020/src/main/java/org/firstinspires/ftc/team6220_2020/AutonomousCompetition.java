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

        waitForStart();

        //for (int i = 0; i < 4; i++) {
        //    driveInches(20, 90);
        //    driveInches(20, 0);
        //    driveInches(20, -90);
        //    driveInches(20, 180);
        //}

        telemetry.addData("Num rings : ", findRingStackSize(2000));
        telemetry.update();
        findRingStackSize(2000);

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        sleep(5000);

        //driveInches(20, 0);

        //driveInches(20, -90);

        //driveLauncher(0.8);

        //for(int i = 0; i < 3; i++){
        //    fireLauncher(1500);
        //    pauseMillis(1500);
        //}
    }
}