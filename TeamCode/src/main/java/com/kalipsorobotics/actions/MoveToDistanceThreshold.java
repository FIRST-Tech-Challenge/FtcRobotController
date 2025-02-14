package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class MoveToDistanceThreshold extends Action {

    private final DriveTrain driveTrain;

    private final DistanceDetectionAction distanceDetectionAction;
    private final double power;

    public MoveToDistanceThreshold(DriveTrain driveTrain, DistanceDetectionAction distanceDetectionAction,
                                   double power) {

        this.driveTrain = driveTrain;
        this.distanceDetectionAction = distanceDetectionAction;
        this.power = power;

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

        if (distanceDetectionAction.checkDistance()) {
            finishedMoving();
        } else {
            driveTrain.setPower(power);
        }


    }
}
