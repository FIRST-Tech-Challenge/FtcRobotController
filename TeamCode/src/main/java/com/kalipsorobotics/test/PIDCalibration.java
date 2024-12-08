package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.PID.PIDController;

import com.kalipsorobotics.actions.drivetrain.DriveTrainAction;
import com.kalipsorobotics.actions.drivetrain.MecanumRobotAction;
import com.kalipsorobotics.actions.drivetrain.MoveRobotStraightInchesAction;

import com.kalipsorobotics.actions.intake.MoveIntakeLSAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.OpModeUtilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PIDCalibration")
public class PIDCalibration extends LinearOpMode {
    static double learningRateP = 0.05;
    static double learningRateI = 0.002;
    static double learningRateD = 0.02;
    static double learningAdjustmentRate = -0.1;
    static double timeDecay = 0.98;
    static double overshootThreshold = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, 0);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, Math.toRadians(0));

        DriveTrainAction action = new MoveRobotStraightInchesAction(24, driveTrain, sparkfunOdometry, wheelOdometry, 0, 5);
        PIDController globalController = action.getPidController();
        String tag = String.format("ILC %s", globalController.getName());

//        MoveIntakeLSAction maintainIntake = new MoveIntakeLSAction(new Intake(opModeUtilities), 0);

        int i = 0;

        double accumulatedError = 0;
        double prevSetDuration = Integer.MAX_VALUE;
        double totalSetDuration = 0;
        double totalSetError = 0;
        double totalOvershoot = 0;

        waitForStart();
        while (opModeIsActive()) {
            wheelOdometry.updatePosition();
            action.updateCheckDone();

            if (action.getIsDone()) {
                i++;

                double error = Math.abs(action.getRemainingDistance());
                totalSetError += error;
                accumulatedError += error;
                totalSetDuration += action.getDuration();
                totalOvershoot += action.getOvershoot();

                double errorRate = totalSetError / totalSetDuration;

                if (i % 2 == 0) {
                    if (totalSetDuration < prevSetDuration) {  // Faster-- decrease learning rates to prevent overly high increases
                        prevSetDuration = totalSetDuration;
                        learningRateP *= Math.exp(learningAdjustmentRate);
                        learningRateI *= Math.exp(learningAdjustmentRate);
                        learningRateD *= Math.exp(learningAdjustmentRate);
                    }
                    if (totalOvershoot > overshootThreshold) {
                        learningRateP *= Math.exp(learningAdjustmentRate);
                        learningRateI *= Math.exp(learningAdjustmentRate);
                        learningRateD *= Math.exp(-learningAdjustmentRate * Math.max(1.1, Math.sqrt(totalOvershoot)));  // Increase D to dampen faster
                    }

                    Log.d(tag, String.format("Prev set duration %f, current set duration %f, total overshoot %f", prevSetDuration, totalSetDuration, totalOvershoot));
                    Log.d(tag, String.format("Learning rates modified to %f, %f, %f", learningRateP, learningRateI, learningRateD));

                    double deltaKP = learningRateP * totalSetError;
                    double deltaKI = learningRateI * (accumulatedError / i);
                    double deltaKD = learningRateD * errorRate;

                    globalController.chKp(deltaKP);
                    globalController.chKi(deltaKI);
                    globalController.chKd(deltaKD);

                    globalController.setKp(globalController.chKp(0) * timeDecay);
                    globalController.setKi(globalController.chKi(0) * timeDecay);
                    globalController.setKd(globalController.chKd(0) * timeDecay);

                    totalSetError = 0;
                    totalSetDuration = 0;
                    totalOvershoot = 0;
                }

                sleep(500);  // should be safe I think
//                maintainIntake.update();

                action = new MoveRobotStraightInchesAction(i % 2 == 0 ? 24 : -24, driveTrain, sparkfunOdometry, wheelOdometry, 0, 5);
                action.setPidController(globalController);

                Log.d(tag, String.format("Error %f, Accumulated error %f, Error rate %f", totalSetError, accumulatedError, errorRate));
                Log.d(tag, globalController.toString());

//                        Log.d(tag, String.format("Kp increased by learning rate %f * error %f = %f", learningRateP, error, deltaKP));
//                        Log.d(tag, String.format("Ki increased by learning rate %f * accumulated error %f = %f", learningRateI, accumulatedError, deltaKI));
//                        Log.d(tag, String.format("Kd increased by learning rate %f * error rate %f = %f", learningRateD, errorRate, deltaKD));

            }
        }
    }
}
