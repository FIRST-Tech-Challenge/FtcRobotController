package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Duck", group = "Autonomous")
@Disabled
public class blueDuck extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();
        pauseMillis(500);
        driveInches(3, 0.5);
        turnDegrees(90, 0.6);
        driveInches(19, 0.8);
        driveInches(0.5, 0.3);
        blueDuckTask();
        driveInches(-3, 0.5);
        turnDegrees(80, 0.6);
        driveInches(20, 0.6);
    }
}