package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;

public class MoveRobotStraightInchesAction extends Action {
    private static final double ERROR_TOLERANCE = 20;
    DriveTrain driveTrain;
    PIDController straightController;
    double targetTicks;
    double currentTicks;
    double error;
    CalculateTickInches calculateTickInches = new CalculateTickInches();

    public MoveRobotStraightInchesAction(Action dependentAction, double targetInches, DriveTrain driveTrain) {
        this.dependentAction = dependentAction;
        straightController = new PIDController("straight", 0.0005, 0.0000015, 0.8, false);
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(targetInches);
        this.driveTrain = driveTrain;
    }

    public MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain) {
        this.dependentAction = new DoneStateAction();
        straightController = new PIDController("straight", 0.0005, 0.0000015, 0.8, false);
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(targetInches);
        this.driveTrain = driveTrain;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    boolean checkDoneCondition() {
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
    void update() {

        this.currentTicks = driveTrain.getfLeftTicks();
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
