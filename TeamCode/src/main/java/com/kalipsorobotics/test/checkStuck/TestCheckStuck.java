package com.kalipsorobotics.test.checkStuck;

import android.os.SystemClock;

import com.kalipsorobotics.actions.CheckStuckRobot;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestCheckStuck extends LinearOpMode {
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
            Path path = null; //TODO find way to get path
            int currentTime = (int) SystemClock.currentThreadTimeMillis();
            checkStuck.isStuck(/*path*/currentTime);
            driveAction.move(gamepad1);
        }
    }
}
