package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
public class OtherRobotTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);
        waitForStart();
        while (opModeIsActive()) {
            driveAction.move(gamepad1);
        }
    }
}
