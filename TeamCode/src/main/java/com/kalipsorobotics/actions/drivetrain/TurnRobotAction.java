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

    DriveTrain driveTrain;
    PIDController controller;
    SparkfunOdometry sparkfunOdometry;
    WheelOdometry wheelOdometry;
    double targetDegrees;
    double currentHeading;
    double ERROR_TOLERANCE = 0.5; // degrees
    double remainingDegrees;
    double startTime;
    double timeout;
    double duration;

    public TurnRobotAction(double targetDegrees, DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry, WheelOdometry wheelOdometry, double timeout) {
        this.dependentActions.add(new DoneStateAction());
        this.targetDegrees = targetDegrees;
        this.driveTrain = driveTrain;

        this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;

        this.controller = new PIDController(0.2, 0.01, 0.01, "turn");  // placeholder values
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
        return Math.toDegrees(wheelOdometry.getCurrentImuHeading());
    }

    private void refreshError() {
        remainingDegrees = MathFunctions.angleWrapDeg(targetDegrees - currentHeading);
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        duration = (SystemClock.elapsedRealtime() - startTime) / 1000;
        if ((Math.abs(remainingDegrees) <= ERROR_TOLERANCE) || duration > timeout) {
            driveTrain.setPower(0, 0, 0, 0);
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
//            currentHeading = getCurrentHeading();
//            targetDegrees = 0;
            Log.d("turn", "setHeading: final heading is " + currentHeading);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        double power;
        double minPower = 0.15;

        Log.d("turn/error", String.format("Error %f Current Theta %f Target Theta %f", remainingDegrees, currentHeading, targetDegrees));

        currentHeading = getCurrentHeading();
        if (!hasStarted) {
            this.targetDegrees += currentHeading;
            Log.d("turn", "target degrees is " + this.targetDegrees);
            hasStarted = true;
            startTime = SystemClock.elapsedRealtime();
        }

        refreshError();
        power = Range.clip(controller.calculate(currentHeading, targetDegrees), -0.7, 0.7);

        if (power > 0 && power < minPower) {
            power = minPower;
        } else if (power < 0 && power > -1 * minPower) {
            power = -minPower;
        }

        Log.d("turn/power", String.format("Turning power %f", power));
        driveTrain.setPower(-power, power, -power, power);
    }
}
