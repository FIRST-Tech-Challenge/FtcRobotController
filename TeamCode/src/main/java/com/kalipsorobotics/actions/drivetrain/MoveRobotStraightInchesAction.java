package com.kalipsorobotics.actions.drivetrain;

import android.util.Log;

import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveRobotStraightInchesAction extends DriveTrainAction {
    private static final double ERROR_TOLERANCE = 0.2;
    DriveTrain driveTrain;
    Odometry odometry;
    PIDController pidController;
    double targetInches;
    double currentInches;
    double remainingDistance;

    public MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain, Odometry odometry) {
        this.dependentAction = new DoneStateAction();
        this.pidController = new PIDController(0.101562, -0.008385, -0.000006, "straight");
        this.driveTrain = driveTrain;
        this.odometry = odometry;
        this.targetInches = targetInches;
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

    @Override
    public boolean checkDoneCondition() {
        refreshRemainingDistance();
        Log.d("straight", "current error is " + remainingDistance);
        if (Math.abs(remainingDistance) <= ERROR_TOLERANCE) {
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
        this.currentInches = odometry.countX();
        refreshRemainingDistance();

        if(!hasStarted) {
            this.targetInches += currentInches;
            Log.d("straight","target inches is " + this.targetInches);
            hasStarted = true;
        }

        if(!getIsDone()){
            double power = pidController.calculate(remainingDistance);
            if (Math.abs(power) < minPower) {
                power = power < 0 ? -minPower : minPower;
            }
            driveTrain.setPower(power);
        }
        else {
            driveTrain.setPower(0);
        }
    }
}
