package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Calibrate Straight")
public class CalibrateStraight extends BaseAutonomous {
    public void runOpMode() {
        initializeAuto();
        waitForStart();
        driveInches(0, 48);

        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}

