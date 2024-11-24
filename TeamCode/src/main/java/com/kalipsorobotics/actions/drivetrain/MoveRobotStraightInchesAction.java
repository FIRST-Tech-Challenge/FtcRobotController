package com.kalipsorobotics.actions.drivetrain;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveRobotStraightInchesAction extends DriveTrainAction {
    private static final double ERROR_TOLERANCE = 0.2;
    DriveTrain driveTrain;
    Odometry odometry;
    PIDController controller;
    double targetInches;
    double currentInches;
    double error;

    public MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain, Odometry odometry) {
        this.dependentAction = new DoneStateAction();
        this.controller = new PIDController(0.0005, 0.0000015, 0.8, "straight");
        this.driveTrain = driveTrain;
        this.odometry = odometry;
        this.targetInches = targetInches;
    }

    public PIDController getController() {
        return controller;
    }

    public double getError() {
        return error;
    }

    private void refreshError() {
        error = targetInches - currentInches;
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        Log.d("straight", "current error is " + error);
        if (Math.abs(error) <= ERROR_TOLERANCE) {
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
        this.currentInches = odometry.countY();
        refreshError();

        if(!hasStarted) {
            this.targetInches += currentInches;
            Log.d("straight","target inches is " + this.targetInches);
            hasStarted = true;
        }

        if(!getIsDone()){
            driveTrain.setPower(controller.calculate(error));
        }
    }
}
