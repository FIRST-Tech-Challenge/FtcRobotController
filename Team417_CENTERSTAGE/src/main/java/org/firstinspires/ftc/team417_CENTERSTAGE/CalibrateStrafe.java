package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Calibrate Strafe")
public class CalibrateStrafe extends BaseAutonomous {
    public void runOpMode() {
        initializeAuto();
        waitForStart();
        driveInches(48, 0);

        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}
