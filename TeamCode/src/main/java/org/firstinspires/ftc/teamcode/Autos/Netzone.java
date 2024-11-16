package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "Netzone", group = "BlueAutos")
public class Netzone extends LinearOpMode {
    private double DRIVE_SPEED = .9;
    private double TURN_SPEED = .5;

    private double SIDE_SPEED = .8;

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
        sleep(100);
        drivebase.driveSideways(SIDE_SPEED, 10, 90);
        drivebase.turnToHeading(TURN_SPEED, 75);
        drivebase.driveStraight(DRIVE_SPEED, 49, 75);
        sleep(100);

        //Transition between first score and second score.
        drivebase.turnToHeading(TURN_SPEED, 75);
        drivebase.driveStraight(DRIVE_SPEED, -46, 75);
        //Was 14.
        sleep(100);
        drivebase.driveSideways(SIDE_SPEED, 17, 75);

        //Score second.
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(DRIVE_SPEED, 50, 90);

        //Score third.
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(DRIVE_SPEED, -50, 90);
        //sleep(500);
        //Was 10.
        sleep(200);
        //Hit wall.
        drivebase.driveSideways(.2, 8.0, 90);
        //drivebase.turnToHeading(TURN_SPEED, 92);
        drivebase.driveStraight(DRIVE_SPEED, 42, 90);

        //Park
        //drivebase.turnToHeading(TURN_SPEED, 90);
        //Was -16.
        drivebase.driveStraight(DRIVE_SPEED, -12, 90);
        //Get away from the wall before turning.
        //sleep(500);
        //Was -8.
        drivebase.driveSideways(SIDE_SPEED, -8, 90);
        //Continue parking.
        drivebase.turnToHeading(TURN_SPEED, 180);
        drivebase.driveStraight(1, 104, 180);
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(0.6, 19, 90);
        //sleep(500);
        //drivebase.driveSideways(SIDE_SPEED, 22, 180);
    }
}
