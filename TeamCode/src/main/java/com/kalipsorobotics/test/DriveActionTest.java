package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.DriveAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
@TeleOp
public class DriveActionTest extends LinearOpMode {
    private LinearOpMode opMode;
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap,opMode,telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);
        waitForStart();
        while (opModeIsActive()) {
            driveAction.move(gamepad1);
        }
    }
}