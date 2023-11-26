package org.firstinspires.ftc.team417_CENTERSTAGE.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.League1BaseAutonomous;

@Autonomous (name = "Calibrate Straight")
public class CalibrateStraight extends League1BaseAutonomous {
    public void runOpMode() {
        initializeAuto();
        waitForStart();
        driveInches(0, 48);

        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}

