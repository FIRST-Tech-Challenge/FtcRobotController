package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "Netzone", group = "BlueAutos")
public class Netzone extends LinearOpMode {
    private double DRIVE_SPEED = .75;
    private double TURN_SPEED = .5;

    private double SIDE_SPEED = .25;

    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        //Score preloaded sample.
        drivebase.driveSideways(SIDE_SPEED, 4, 0);
        drivebase.turnToHeading(TURN_SPEED, -15);
        //Was 22 before.
        drivebase.driveStraight(DRIVE_SPEED, 18, -15);
        drivebase.turnToHeading(TURN_SPEED, 45);
        drivebase.driveStraight(DRIVE_SPEED, 5, 45);

        //Score 1st neutral sample.
        drivebase.driveStraight(DRIVE_SPEED, -24, 45);
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(DRIVE_SPEED, -32, 90);
        drivebase.driveSideways(SIDE_SPEED, 10, 90);
        drivebase.turnToHeading(TURN_SPEED, 75);
        drivebase.driveStraight(DRIVE_SPEED, 49, 75);
        sleep(500);

        //Transition between first score and second score.
        drivebase.driveStraight(DRIVE_SPEED, -46, 75);
        drivebase.driveSideways(SIDE_SPEED, 14, 75);

        //Score second.
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(DRIVE_SPEED, 44, 90);

        //Score third.
        drivebase.driveStraight(DRIVE_SPEED, -44, 90);
        drivebase.driveSideways(SIDE_SPEED, 14, 90);
        drivebase.driveStraight(DRIVE_SPEED, 42, 90);

        //Park
        drivebase.driveStraight(DRIVE_SPEED, -4, 90);
        drivebase.turnToHeading(TURN_SPEED, 180);
        drivebase.driveStraight(1, 112, 180);
        drivebase.driveSideways(SIDE_SPEED, 14, 90);
    }
}
