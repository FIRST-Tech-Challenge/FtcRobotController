package com.kalipsorobotics.actions;

import android.util.Log;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveRobotStraightInchesAction extends Action {
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

    private void refreshError() {
        error = targetInches - currentInches;
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        Log.d("moverobot", "current error is " + error);
        if (Math.abs(error) <= ERROR_TOLERANCE) {
            driveTrain.setPower(0);
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            Log.d("moverobot","isdone true");
            return true;
        } else {
            Log.d("moverobot","isdone false");
            return false;
        }
    }

    @Override
    public void update() {
        this.currentInches = odometry.countY();
        refreshError();

        if(!hasStarted) {
            this.targetInches += currentInches;
            Log.d("moverobot","target ticks is" + this.targetInches);
            hasStarted = true;
        }

        if(!isDone){
            driveTrain.setPower(controller.calculate(error));
        }
    }
}
