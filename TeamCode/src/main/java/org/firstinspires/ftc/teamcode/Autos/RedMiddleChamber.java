package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "RedMiddleChamber", group = "BlueAutos")
public class RedMiddleChamber extends LinearOpMode {
    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Score preloaded specimen.
        drivebase.driveStraight(0.3, 26.0, 0);

        //To put 1st sample into obs zone.
        drivebase.turnToHeading(.3, -90);
        //drivebase.holdHeading(.3, -90, 0.1);
        drivebase.driveStraight(.4, 22, -90);
        drivebase.turnToHeading(0.3, 0);
        //drivebase.holdHeading(.3, 0, 0.1);

        drivebase.driveStraight(0.5, 28, 0);
        drivebase.turnToHeading(0.4, -90);
        drivebase.driveStraight(0.5, 10, -90);
        drivebase.turnToHeading(0.4, -180);
        drivebase.driveStraight(0.5,46, -180);
        drivebase.driveStraight(0.5, -12, -180);
        sleep(2000);
        drivebase.turnToHeading(0.4, -90);
        drivebase.driveStraight(0.5, 14,-90);
        drivebase.turnToHeading(0.4, -180);
        drivebase.driveStraight(0.5, 14,-180);
        drivebase.turnToHeading(0.4, -270);
        drivebase.driveStraight(0.5, 48, -270);


        /*
        //Transition from current line to next one down.
        drivebase.turnToAngle(-180, 0.3, telemetry);

        //Grab second piece and put into obs zone.
        drivebase.driveSideways(0.3, 12);
        drivebase.autoDriveForward(36.0, 0.3);
        drivebase.turnToAngle(-180, 0.3, telemetry);
        drivebase.autoDriveForward(0.3, 48.0);

        //Transition to score 1st human player made specimen onto the chamber.
        drivebase.turnToAngle(90, 0.1, telemetry);

        //Score first human made specimen onto chamber.
        drivebase.autoDriveForward(0.3, 36);
        drivebase.turnToAngle(90.0, 0.2, telemetry);
        drivebase.autoDriveForward(0.3, 48);

        //Score second human player made specimen.
        drivebase.turnToAngle(-35, 0.3, telemetry);
        drivebase.autoDriveForward(0.3, -60);
        drivebase.autoDriveForward(0.3, 60);

        //Park in obs zone.
        drivebase.autoDriveForward(0.3, -60);
        */
    }
}
