package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "Netzone", group = "BlueAutos")
public class Netzone extends LinearOpMode {
    private double DRIVE_SPEED = .5;
    private double TURN_SPEED = .5;

    private double SIDE_SPEED = .2;

    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        //Score preloaded sample.
        drivebase.driveSideways(SIDE_SPEED, 4, 0);
        drivebase.turnToHeading(TURN_SPEED, -15);
        drivebase.driveStraight(DRIVE_SPEED, 22, -15);
        drivebase.turnToHeading(TURN_SPEED, 45);
        drivebase.driveStraight(DRIVE_SPEED, 3, 45);

        //Score 1st neutral sample.
        drivebase.driveStraight(DRIVE_SPEED, -30, 45);
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(DRIVE_SPEED, -27, 90);
        drivebase.turnToHeading(TURN_SPEED, 0);
        drivebase.driveStraight(DRIVE_SPEED, 5, 0);
        drivebase.turnToHeading(TURN_SPEED, 75);
        drivebase.driveStraight(DRIVE_SPEED, 50, 75);
        sleep(500);

        //Transition between first score and second score.
        drivebase.driveStraight(DRIVE_SPEED, -46, 75);
        drivebase.turnToHeading(TURN_SPEED, 0);

        //Score second.
        drivebase.driveStraight(DRIVE_SPEED, 13, 0);
        drivebase.turnToHeading(TURN_SPEED, 90);
        drivebase.driveStraight(DRIVE_SPEED, 42, 90);
    }
}
