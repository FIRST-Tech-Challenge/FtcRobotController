package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivebase;

@Autonomous(name = "BlueBasket", group = "BlueAutos")
public class BlueBasket extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivebase drivebase = new Drivebase(hardwareMap, this::opModeIsActive, telemetry);

        waitForStart();

        //Testing statement.
        drivebase.driveStraight(0.5, 10,0);

        //Score preloaded specimen into basket.
        //drivebase.autoDriveForward(0.3, 10);
        /*
        drivebase.turnToAngle(0.3, -90.0, telemetry);
        drivebase.autoDriveForward(0.03, 36);

        //Score first sample into basket. (Top spike mark).
        drivebase.turnToAngle(0.3, 90, telemetry);
        drivebase.autoDriveForward(0.3,36);
        drivebase.turnToAngle(0.3, -180, telemetry);
        drivebase.autoDriveForward(0.3, 36);
        drivebase.turnToAngle(0.3, -180, telemetry);

        //Score second sample into basket. (middle)
        drivebase.driveSideways(0.3, 10);
        drivebase.autoDriveForward(0.3, 36);
        drivebase.turnToAngle(0.3, -180, telemetry);
        drivebase.autoDriveForward(0.3, 36);
        drivebase.turnToAngle(0.3, -180, telemetry);

        //Score third sample into basket. (last).
        drivebase.driveSideways(0.3, 10);
        drivebase.autoDriveForward(0.3, 36);
        drivebase.turnToAngle(0.3, -180, telemetry);
        drivebase.autoDriveForward(0.3, 36);
        drivebase.turnToAngle(0.3, -180, telemetry);

        //Park in obs zone.
        drivebase.turnToAngle(0.3, 45, telemetry);
        drivebase.autoDriveForward(0.3, 66.4830805544);
         */
    }
}
