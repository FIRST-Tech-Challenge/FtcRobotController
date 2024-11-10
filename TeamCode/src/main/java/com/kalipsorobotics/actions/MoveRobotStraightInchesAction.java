package com.kalipsorobotics.actions;

import android.util.Log;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.math.CalculateTickInches;
import com.kalipsorobotics.PID.PIDController2023;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveRobotStraightInchesAction extends Action {
    private static final double ERROR_TOLERANCE = 20;
    DriveTrain driveTrain;
    Odometry odometry;
    PIDController2023 straightController;
    double targetTicks;
    double currentTicks;
    double error;
    CalculateTickInches calculateTickInches = new CalculateTickInches();

    public MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain) {
        this.dependentAction = new DoneStateAction();
        straightController = new PIDController2023("straight", 0.0005, 0.0000015, 0.8, false);
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(targetInches);
        this.driveTrain = driveTrain;
        this.odometry = odometry;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
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

        this.currentTicks = odometry.countRight();
        Log.d("moverobot", "current ticks is " + currentTicks);
        Log.d("moverobot", "target inches is " + calculateTickInches.ticksToInchesDriveTrain(targetTicks));

        if(!hasStarted) {
            this.targetTicks += currentTicks;
            Log.d("moverobot","target ticks is" + this.targetTicks);
            hasStarted = true;
        }

        if(!isDone){
            driveTrain.setPower(straightController.calculatePID(currentTicks, targetTicks));
        }
    }
}
