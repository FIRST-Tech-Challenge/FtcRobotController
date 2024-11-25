package com.kalipsorobotics.actions.drivetrain;

import android.util.Log;

import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveRobotStraightInchesAction extends DriveTrainAction {
    private static final double ERROR_TOLERANCE = 0.2;
    DriveTrain driveTrain;
    SparkfunOdometry sparkfunOdometry;
    WheelOdometry wheelOdometry;
    PIDController pidController;
    double targetInches;
    double currentInches;
    double remainingDistance;
    double prevThetaRadians;
    double targetTheta;
    double thetaOffset;
    double rotationPower;

    public MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry, WheelOdometry wheelOdometry, double targetTheta) {
        this.dependentAction = new DoneStateAction();
        this.pidController = new PIDController(0.309011, 0.000380, 0.000015, "straight");
        this.driveTrain = driveTrain;
        this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;
        this.targetInches = targetInches;
        this.targetTheta = targetTheta;
        prevThetaRadians = sparkfunOdometry.countTheta();
    }

    public PIDController getPidController() {
        return pidController;
    }

    public void setPidController(PIDController controller) {
        pidController = controller;
    }

    public double getRemainingDistance() {
        return remainingDistance;
    }

    public double getTarget() {
        return targetInches;
    }

    private void refreshRemainingDistance() {
        remainingDistance = targetInches - currentInches;
    }

    private void refreshThetaOffset() {thetaOffset = targetTheta - sparkfunOdometry.countTheta();}

    @Override
    public boolean checkDoneCondition() {
        refreshRemainingDistance();
        Log.d("straight", "current error is " + remainingDistance);
        if (Math.abs(remainingDistance) <= ERROR_TOLERANCE && Math.abs(thetaOffset) <= Math.toRadians(1)) {
            driveTrain.setPower(0);
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            Log.d("straight","isdone true");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        double minPower = 0.2;
        this.currentInches = wheelOdometry.countLeft();
        refreshRemainingDistance();

        if (!hasStarted) {
            this.targetInches += currentInches;
            Log.d("straight","target inches is " + this.targetInches);
            hasStarted = true;
        }

        double linearPower = pidController.calculate(remainingDistance);
        if (Math.abs(linearPower) < minPower) {
            linearPower = linearPower < 0 ? -minPower : minPower;
        }

        refreshThetaOffset();

        if (Math.abs(thetaOffset) > Math.toRadians(1)) {
            rotationPower = -thetaOffset * 0.25;
        }

        Log.d("ILC str rotation", String.format("Theta offset %s, Rotation Power %s", thetaOffset, rotationPower));

        double fLeft = linearPower + rotationPower;
        double fRight = linearPower - rotationPower;
        double bLeft = linearPower + rotationPower;
        double bRight = linearPower - rotationPower;

        double biggest = MathFunctions.maxAbsValueDouble(fLeft, fRight, bLeft, bRight);
        if (biggest > 1) {
            fLeft /= biggest;
            fRight /= biggest;
            bLeft /= biggest;
            bRight /= biggest;
        }

        driveTrain.setPower(fLeft, fRight, bLeft, bRight);
    }
}
