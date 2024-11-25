package com.kalipsorobotics.test;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.PID.PIDController;

import com.kalipsorobotics.actions.drivetrain.DriveTrainAction;
import com.kalipsorobotics.actions.drivetrain.MoveRobotStraightInchesAction;

import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.utilities.OpModeUtilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="PIDCalibration")
public class PIDCalibration extends LinearOpMode {
    public static final double learningRateP = 0.05;
    public static final double learningRateI = 0.001;
    public static final double learningRateD = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, 0);
        WheelOdometry wheelOdometry = new WheelOdometry(driveTrain, opModeUtilities, imuModule, 0, 0, Math.toRadians(0));

        waitForStart();

        DriveTrainAction action = new MoveRobotStraightInchesAction(24, driveTrain, sparkfunOdometry, wheelOdometry, 0);
        PIDController globalController = action.getPidController();

        int i = 0;
        double accumulatedError = 0.;
        double prevError = 0;
        double prevTime = SystemClock.elapsedRealtime();

        while (opModeIsActive()) {
            sparkfunOdometry.updatePosition();
            action.updateCheckDone();
            if (action.getIsDone()) {
                i++;
                String tag = String.format("ILC %s", globalController.getName());

                double currentTime = SystemClock.elapsedRealtime();
                double error = action.getRemainingDistance();
                accumulatedError += Math.abs(error);
                double errorRate = (error - prevError) / (currentTime - prevTime);

                Log.d(tag, sparkfunOdometry.getCurrentPosition().toString());
                Log.d(tag, String.format("Error %f, Accumulated error %f, Error rate %f", error, accumulatedError, errorRate));

                double deltaKP = learningRateP * error;
                double deltaKI = learningRateI * accumulatedError;
                double deltaKD = learningRateD * errorRate;

                globalController.chKp(deltaKP);
                globalController.chKi(deltaKI);
                globalController.chKd(deltaKD);

                sleep(1000);  // should be safe I think
                action = new MoveRobotStraightInchesAction(i % 2 == 0 ? 24 : -24, driveTrain, sparkfunOdometry, wheelOdometry, 0);
                action.setPidController(globalController);

                Log.d(tag, globalController.toString());

                prevError = error;
                prevTime = currentTime;

//                        Log.d(tag, errorLog.toString());
//                        Log.d(tag, String.format("Kp increased by learning rate %f * error %f = %f", learningRateP, error, deltaKP));
//                        Log.d(tag, String.format("Ki increased by learning rate %f * accumulated error %f = %f", learningRateI, accumulatedError, deltaKI));
//                        Log.d(tag, String.format("Kd increased by learning rate %f * error rate %f = %f", learningRateD, errorRate, deltaKD));

            }
        }
    }
}
