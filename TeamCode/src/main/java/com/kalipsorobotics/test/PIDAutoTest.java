package com.kalipsorobotics.test;

import android.util.Log;

import androidx.annotation.NonNull;

import com.kalipsorobotics.PID.PIDController;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.actions.drivetrain.DriveTrainAction;
import com.kalipsorobotics.actions.drivetrain.MecanumRobotAction;
import com.kalipsorobotics.actions.drivetrain.MoveRobotStraightInchesAction;
import com.kalipsorobotics.actions.drivetrain.TurnRobotAction;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.kuriosityrobotics.shuttle.HardwareTaskScope;

import java.util.ArrayList;
import java.util.HashMap;


@Autonomous(name="PIDAutoTest")
public class PIDAutoTest extends LinearOpMode {
    public static final double learningRateP = 0.0001;
    public static final double learningRateI = 0.00001;
    public static final double learningRateD = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        try (var outer = HardwareTaskScope.open()) {
            OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
            DriveTrain driveTrain = new DriveTrain(opModeUtilities);
            Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, 0);
            outer.fork(odometry::run);

            try (var inner = HardwareTaskScope.open()) {
                ActionSet set = getActionSet(driveTrain, odometry);
                waitForStart();
                inner.fork(set::update);

                HashMap<DriveTrainAction, ArrayList<Double>> errorLogs = new HashMap<>();
                for (Action action : set.getActions()) {
                    errorLogs.put((DriveTrainAction) action, new ArrayList<>());
                }

                while (opModeIsActive()) {
                    for (Action action : set.getActions()) {
                        if (action.getHasStarted() && !action.getIsDone() && action instanceof DriveTrainAction) {
                            DriveTrainAction d = (DriveTrainAction) action;
                            PIDController controller = d.getController();

                            String tag = String.format("ILC %s", controller.getName());

                            double error = d.getError();
                            double deltaTime = odometry.deltaTime();
                            Log.d(tag, String.format("Error %s with deltaTime %s", error, deltaTime));
                            errorLogs.get(d).add(error);

                            double accumulatedError = errorLogs.get(action).stream().mapToDouble(Double::doubleValue).sum();
                            double previousError = errorLogs.get(action).size() > 1
                                    ? errorLogs.get(action).get(errorLogs.get(action).size() - 2)
                                    : 0.0;
                            double errorRate = (error - previousError) / deltaTime;
//                            Log.d(tag, String.format("Accumulated error %f", accumulatedError));
//                            Log.d(tag, String.format("Previous error %f", previousError));
//                            Log.d(tag, String.format("Error rate %f", errorRate));

                            double deltaKP = learningRateP * error;
                            double deltaKI = learningRateI * accumulatedError;
                            double deltaKD = learningRateD * errorRate;
                            Log.d(tag, controller.toString());

                            controller.chKp(deltaKP);
                            controller.chKi(deltaKI);
                            controller.chKd(deltaKD);

                            Log.d(tag, String.format("Kp increased by learning rate %f * error %f = %f", learningRateP, error, deltaKP));
                            Log.d(tag, String.format("Ki increased by learning rate %f * accumulated error %f = %f", learningRateI, accumulatedError, deltaKI));
                            Log.d(tag, String.format("Kd increased by learning rate %f * error rate %f = %f", learningRateD, errorRate, deltaKD));
                            Log.d(tag, controller.toString());
                        }
                    }
                }
                inner.shutdown();
                inner.join();

            } finally {
                outer.shutdown();
                outer.join();
            }
        }
    }

    @NonNull
    private static ActionSet getActionSet(DriveTrain driveTrain, Odometry odometry) {
        MoveRobotStraightInchesAction straightTest = new MoveRobotStraightInchesAction(24, driveTrain, odometry);
        MecanumRobotAction mecanumTest = new MecanumRobotAction(12, driveTrain, odometry);
        TurnRobotAction turnTest = new TurnRobotAction(45, driveTrain, odometry);

        ActionSet set = new ActionSet();
        set.scheduleSequential(straightTest);
        set.scheduleSequential(mecanumTest);
        set.scheduleSequential(turnTest);
        return set;
    }
}
