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
import com.kalipsorobotics.utilities.SharedData;
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
            if (checkStuck.isStuck(currentTime)) {
                telemetry.addLine("robot is stuck");
                //Log.d("check stucks", "ROBOT STUCK");
            } else {
                //Log.d("check stucks", "ROBOT NOT STUCK");
                telemetry.addLine("robot is fine");
            }
            Log.d("check stucks", "x delta: " + checkStuck.getXDelta(SharedData.getOdometryPosition()) +
                    " y delta: " + checkStuck.getYDelta(SharedData.getOdometryPosition()) +
                    " theta delta: " + checkStuck.getThetaDelta(SharedData.getOdometryPosition()));
            driveAction.move(gamepad1);
            telemetry.update();
            //TODO fix shared data thing
        }
    }
}
