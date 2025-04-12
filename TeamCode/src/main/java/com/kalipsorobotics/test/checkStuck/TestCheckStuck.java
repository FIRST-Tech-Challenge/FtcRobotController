package com.kalipsorobotics.test.checkStuck;

import android.os.SystemClock;
import android.util.Log;

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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestCheckStuck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, new Position(0, 0, 0));
        PurePursuitAction purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry, 0, 0);
        DriveAction driveAction = new DriveAction(driveTrain);
        CheckStuckRobot checkStuck = new CheckStuckRobot(driveTrain, wheelOdometry, opModeUtilities, purePursuitAction);
        waitForStart();
        while(opModeIsActive()) {
            Path path = null; //TODO find way to get path AND IMPLEMENT INTO CHECKXY
            int currentTime = (int) System.currentTimeMillis();
            Log.d("check stuck", "current time is: " + currentTime);
            boolean isStuck = checkStuck.isStuck(currentTime);
            if (isStuck) {
                telemetry.addLine("robot is stuck");
            }
            driveAction.move(gamepad1);
            telemetry.update();
        }
    }
}
