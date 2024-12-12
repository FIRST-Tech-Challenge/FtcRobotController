package com.kalipsorobotics.actions.drivetrain;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import com.kalipsorobotics.modules.DriveTrain;

public class TurnRobotAction extends DriveTrainAction {

    private static final double ERROR_TOLERANCE = 0.5; // degrees
    DriveTrain driveTrain;
    PIDController controller;
    SparkfunOdometry sparkfunOdometry;
    WheelOdometry wheelOdometry;

    double targetDegrees;
    double currentHeading;
    double remainingDegrees;

    double startTime;
    double timeout;
    double duration;

    double overshoot;
    private double initialError;

    public TurnRobotAction(double targetDegrees, DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry, WheelOdometry wheelOdometry, double timeout) {
        this.dependentActions.add(new DoneStateAction());
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;

        this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;

        this.controller = new PIDController(0.077573, 0.000254, 0.003647, "turn");
        this.startTime = Integer.MAX_VALUE;
        this.timeout = timeout;
    }

    public PIDController getPidController() {
        return controller;
    }

    public void setPidController(PIDController controller) {
        this.controller = controller;
    }

    public double getRemainingDistance() {
        return remainingDegrees;
    }

    public double getTarget() {
        return targetDegrees;
    }

    public double getCurrentHeading() {
        return Math.toDegrees(wheelOdometry.getCurrentImuHeading());  // Already angle wrapped
    }

    public double getDuration() {
        return duration;
    }

    public double getOvershoot() {
        return overshoot;
    }

    private void refreshError() {
        remainingDegrees = targetDegrees - currentHeading;
        if (Math.signum(remainingDegrees) != Math.signum(initialError)) {
            overshoot = Math.abs(remainingDegrees);
        }
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        duration = (SystemClock.elapsedRealtime() - startTime) / 1000;
        if ((Math.abs(remainingDegrees) <= ERROR_TOLERANCE) || duration > timeout) {
            driveTrain.setPower(0, 0, 0, 0);
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            Log.d("turn", "setHeading: final heading is " + currentHeading);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        double power;
        double minPower = 0.1;

        currentHeading = getCurrentHeading();
        if (!hasStarted) {
            this.targetDegrees += currentHeading;
            initialError = targetDegrees - currentHeading;
            Log.d("turn", "target degrees is " + this.targetDegrees);

            hasStarted = true;
            startTime = SystemClock.elapsedRealtime();
        }

        refreshError();
        Log.d("turn/error", String.format("Error %f Current Degrees %f Target Degrees %f", remainingDegrees, currentHeading, targetDegrees));

        power = Range.clip(controller.calculate(currentHeading, targetDegrees), -0.7, 0.7);
        if (power > 0 && power < minPower) {
            power = minPower;
        } else if (power < 0 && power > -minPower) {
            power = -minPower;
        }

        Log.d("turn/power", String.format("Turning power %f", power));
        driveTrain.setPower(power, -power, power, -power);
    }
}
