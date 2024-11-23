package com.kalipsorobotics.actions;

import android.util.Log;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.CalculateTickInches;
import com.kalipsorobotics.PID.PIDController2023;
import com.kalipsorobotics.modules.DriveTrain;

public class MecanumRobotAction extends Action {

    private static final double ERROR_TOLERANCE = 0.2;
    DriveTrain driveTrain;
    Odometry odometry;
    PIDController controller;
    double targetInches;
    double currentInches;
    double error;
    double power;

    public MecanumRobotAction(double targetInches, DriveTrain driveTrain, Odometry odometry) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        controller = new PIDController(0.005, 0.0000005, 0.4, "mecanum");
        this.odometry = odometry;
        this.targetInches = targetInches;
    }

    private void refreshError() {
        error = targetInches - currentInches;
    }

    @Override
    public boolean checkDoneCondition() {
        refreshError();
        if(Math.abs(error) <= ERROR_TOLERANCE) {
            driveTrain.setPower(0); // stop, to be safe
            driveTrain.getOpModeUtilities().getOpMode().sleep(100);
            Log.d("mecanum", "done");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        this.currentInches = odometry.countX();
        refreshError();

        if (!hasStarted) {
            this.targetInches += currentInches;
            Log.d("mecanum", "target ticks " + targetInches);
            hasStarted = true;
        }

        if(!isDone){
            driveTrain.setPower(power, -power, -power, power);
        }
    }
}
