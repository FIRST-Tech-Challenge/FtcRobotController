package org.firstinspires.ftc.teamcode.NewStuff;

import com.kalipsorobotics.fresh.DriveTrain;

public class MoveRobotStraightInchesAction extends Action {
    private static final double ERROR_TOLERANCE = 50;
    DriveTrain driveTrain;
    PIDController straightController;
    double targetTicks;
    double currentTicks;
    double error;
    CalculateTickInches calculateTickInches;

    MoveRobotStraightInchesAction(Action dependentAction, double targetInches, DriveTrain driveTrain) {
        this.dependentAction = dependentAction;
        straightController = new PIDController("straight", 0.005, 0.0000015, 0.8, false);
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(targetInches);
        this.driveTrain = driveTrain;
    }

    MoveRobotStraightInchesAction(double targetInches, DriveTrain driveTrain) {
        this.dependentAction = new DoneStateAction();
        straightController = new PIDController("straight", 0.005, 0.0000015, 0.8, false);
        this.targetTicks = calculateTickInches.inchToTicksDriveTrain(targetInches);
        this.driveTrain = driveTrain;
    }

    private void refreshError() {
        error = targetTicks - currentTicks;
    }

    @Override
    boolean checkDoneCondition() {
        refreshError();
        if (error <= ERROR_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    void update() {
        this.currentTicks = driveTrain.getfLeftTicks();
        driveTrain.setPower(straightController.calculatePID(currentTicks, targetTicks));
    }
}
