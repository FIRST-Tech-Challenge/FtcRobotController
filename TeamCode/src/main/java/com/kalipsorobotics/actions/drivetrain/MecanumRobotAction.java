package com.kalipsorobotics.actions.drivetrain;

import android.util.Log;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.modules.DriveTrain;

public class MecanumRobotAction extends DriveTrainAction {  // todo use back wheel

    private static final double ERROR_TOLERANCE = 0.2;
    DriveTrain driveTrain;
    SparkfunOdometry sparkfunOdometry;
    PIDController controller;
    double targetInches;
    double currentInches;
    double remainingDistance;
    double power;

    public MecanumRobotAction(double targetInches, DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry) {
        this.dependentAction = new DoneStateAction();
        this.driveTrain = driveTrain;
        controller = new PIDController(0.005, 0.0000005, 0.4, "mecanum");
        this.sparkfunOdometry = sparkfunOdometry;
        this.targetInches = targetInches;
    }

    public PIDController getPidController() {
        return controller;
    }

    public void setPidController(PIDController controller) {
        this.controller = controller;
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
        if(Math.abs(remainingDistance) <= ERROR_TOLERANCE) {
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
        this.currentInches = sparkfunOdometry.countX();
        refreshRemainingDistance();

        if (!hasStarted) {
            this.targetInches += currentInches;
            Log.d("mecanum", "target ticks " + targetInches);
            hasStarted = true;
        }

        if(!getIsDone()){
            driveTrain.setPower(power, -power, -power, power);
        }
        else {
            driveTrain.setPower(0);
        }
    }
}
