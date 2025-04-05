package com.kalipsorobotics.test.checkStuck;

import com.kalipsorobotics.actions.CheckStuckRobot;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestUnstuck extends LinearOpMode {
    OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
    IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
    DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
    WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, new Position(0, 0, 0));
    PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry, 0, 0);
    DriveAction driveAction = new DriveAction(driveTrain);
    CheckStuckRobot checkStuck = new CheckStuckRobot(driveTrain, wheelOdometry, opModeUtilities, purePursuitAction);
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            checkStuck.isStuck();
            driveAction.move(gamepad1);
        }
    }
}
