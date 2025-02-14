package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PurePursuitDistanceSensorCorrection extends Action {

    Rev2mDistanceSensor revDistance;
    DistanceDetectionAction detectDistanceAction;
    PurePursuitAction purePursuitAction;
    MoveToDistanceThreshold moveToDistanceThreshold;
    WaitAction timeout;
    Telemetry telemetry;

    public PurePursuitDistanceSensorCorrection(Outtake outtake, PurePursuitAction purePursuitAction, DriveTrain driveTrain) {

        this.revDistance = outtake.getRevDistanceBottom();
        detectDistanceAction = new DistanceDetectionAction(revDistance, 155);
        detectDistanceAction.setName("detectDistanceAction");

        this.purePursuitAction = purePursuitAction;

        moveToDistanceThreshold = new MoveToDistanceThreshold(driveTrain, detectDistanceAction, -0.4);
        moveToDistanceThreshold.setName("moveToDistanceThreshold");
        moveToDistanceThreshold.setDependentActions(purePursuitAction);

        timeout = new WaitAction(6000);
    }

    @Override
    protected boolean checkDoneCondition() {
        return moveToDistanceThreshold.getIsDone() &&
                detectDistanceAction.getIsDone() &&
                purePursuitAction.getIsDone();
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }

        if (purePursuitAction.getHasStarted()) {
            purePursuitAction.updateCheckDone();
            detectDistanceAction.updateCheckDone();
            moveToDistanceThreshold.updateCheckDone();
            timeout.updateCheckDone();

            if (timeout.getIsDone()) {
                purePursuitAction.finishedMoving();
                detectDistanceAction.setIsDone(true);
                moveToDistanceThreshold.finishedMoving();
                setIsDone(true);
                return;
            }

            if (detectDistanceAction.getIsDone()) {
                purePursuitAction.finishedMoving();
                Log.d("cancelPurePursuit", "Wall " + revDistance.getDistance(DistanceUnit.MM));
                moveToDistanceThreshold.finishedMoving();
            }

        }
    }
}
