package org.firstinspires.ftc.team417_CENTERSTAGE.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.League1BaseAutonomous;

@Autonomous (name = "Calibrate Strafe")
public class CalibrateStrafe extends League1BaseAutonomous {
    public void runOpMode() {
        initializeAuto();
        waitForStart();
        driveInches(48, 0);

        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}
