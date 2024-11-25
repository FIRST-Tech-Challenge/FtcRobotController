package com.kalipsorobotics.test;

import android.os.SystemClock;
import android.util.Log;

import androidx.annotation.NonNull;

import com.kalipsorobotics.PID.PIDController;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.actions.drivetrain.DriveTrainAction;
import com.kalipsorobotics.actions.drivetrain.MoveRobotStraightInchesAction;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.HashMap;


@Autonomous(name="PIDCalibration")
public class PIDCalibration extends LinearOpMode {
    public static final double learningRateP = 0.05;
    public static final double learningRateI = 0.001;
    public static final double learningRateD = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, 0);

        ActionSet set = getActionSet(driveTrain, odometry);
        waitForStart();

        HashMap<DriveTrainAction, ArrayList<Double>> errorLogs = new HashMap<>();
        for (Action action : set.getActions()) {
            errorLogs.put((DriveTrainAction) action, new ArrayList<>());
        }

        DriveTrainAction action = new MoveRobotStraightInchesAction(24, driveTrain, odometry);
        PIDController globalController = action.getPidController();
        int i = 0;
        double accumulatedError = 0.;
        double prevError = 0;
        double prevTime = SystemClock.elapsedRealtime();

        while (opModeIsActive()) {
            odometry.updatePosition();
            action.updateCheckDone();
            if (action.getIsDone()) {
                String tag = String.format("ILC %s", globalController.getName());
                i++;

                double currentTime = SystemClock.elapsedRealtime();
                double error = action.getRemainingDistance();
                accumulatedError += Math.abs(error);
                double errorRate = (error - prevError) / (currentTime - prevTime);

                Log.d(tag, odometry.getCurrentPosition().toString());
                Log.d(tag, String.format("Error %f, Accumulated error %f, Error rate %f", error, accumulatedError, errorRate));

                double deltaKP = learningRateP * error;
                double deltaKI = learningRateI * accumulatedError;
                double deltaKD = learningRateD * errorRate;

                globalController.chKp(deltaKP);
                globalController.chKi(deltaKI);
                globalController.chKd(deltaKD);
                action = new MoveRobotStraightInchesAction(i % 2 == 0 ? 24 : -24, driveTrain, odometry);
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

    private double[] calcErrors(DriveTrainAction action, Odometry odometry, HashMap<DriveTrainAction, ArrayList<Double>> errorLogs) {
        double error = action.getRemainingDistance();
        double deltaTime = odometry.deltaTime();
        errorLogs.get(action).add(error);

        ArrayList<Double> errorLog = errorLogs.get(action);
        double accumulatedError = errorLog.stream().mapToDouble(Double::doubleValue).sum();
        double previousError = errorLog.size() > 1
                ? errorLog.get(errorLog.size() - 2)
                : 0.;
        double errorRate = (error - previousError) / deltaTime;
        return new double[]{error, accumulatedError, errorRate};
    }

    @NonNull
    private static ActionSet getActionSet(DriveTrain driveTrain, Odometry odometry) {
        ActionSet set = new ActionSet();
        set.scheduleSequential(new MoveRobotStraightInchesAction(24, driveTrain, odometry));
        set.scheduleSequential(new MoveRobotStraightInchesAction(-24, driveTrain, odometry));
        set.scheduleSequential(new MoveRobotStraightInchesAction(24, driveTrain, odometry));
        set.scheduleSequential(new MoveRobotStraightInchesAction(-24, driveTrain, odometry));

        return set;
    }
}
