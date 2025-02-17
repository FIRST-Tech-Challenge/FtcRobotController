package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class MoveToDistanceThreshold extends Action {

    private final DriveTrain driveTrain;

    private final DistanceDetectionAction distanceDetectionAction;
    private final double power;

    double startTime;

    private double timeoutMM;

    public MoveToDistanceThreshold(DriveTrain driveTrain, DistanceDetectionAction distanceDetectionAction,
                                   double power, double timeoutMM) {

        this.driveTrain = driveTrain;
        this.distanceDetectionAction = distanceDetectionAction;
        this.power = power;
        this.timeoutMM = timeoutMM;

    }

    @Override
    protected boolean checkDoneCondition() {
        return isDone;
    }

    public void finishedMoving() {
        driveTrain.setPower(0);
        isDone = true;
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (!hasStarted) {
            startTime = System.currentTimeMillis();
            hasStarted = true;
        }
        double elapsedTime = System.currentTimeMillis() - startTime;

        if(elapsedTime > timeoutMM) {
            finishedMoving();
            return;
        }

        if (distanceDetectionAction.checkDistance()) {
            finishedMoving();
            return;
        }

        driveTrain.setPower(power);



    }
}
