package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "BlueMiddleChamber", group = "BlueAutos")
public class BlueMiddleChamber extends LinearOpMode {
    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Inverse every turn for red.

        //Score preloaded specimen.
        drivebase.driveStraight(0.3, 24.0, 0);

        //To put 1st sample into obs zone.
        //drivebase.driveSideways(0.3, 24);
        //drivebase.autoDriveForward(0.3, 12);
        //drivebase.turnToHeading(0.3, 90);
        /*
        drivebase.autoDriveForward(0.3, 12);
        drivebase.turnToAngle(-90, 0.3, telemetry);
        drivebase.autoDriveForward(0.3,48);

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
