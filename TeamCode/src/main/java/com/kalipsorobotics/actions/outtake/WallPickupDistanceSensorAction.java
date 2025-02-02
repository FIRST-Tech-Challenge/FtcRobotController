package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WallPickupDistanceSensorAction extends Action {

    Rev2mDistanceSensor revDistance;
    DistanceDetectionAction detectDistanceAction;
    PurePursuitAction purePursuitAction;
    KServoAutoAction outtakeClawClose;
    MoveToDistanceThreshold moveToDistanceThreshold;

    public WallPickupDistanceSensorAction(Outtake outtake, PurePursuitAction purePursuitAction, DriveTrain driveTrain) {
        this.revDistance = outtake.revDistanceClaw;
        detectDistanceAction = new DistanceDetectionAction(revDistance, 48);
        detectDistanceAction.setName("detectDistanceAction");

        this.purePursuitAction = purePursuitAction;

        moveToDistanceThreshold = new MoveToDistanceThreshold(driveTrain, detectDistanceAction, -0.2);
        moveToDistanceThreshold.setName("moveToDistanceThreshold");
        moveToDistanceThreshold.setDependentActions(purePursuitAction);

        outtakeClawClose = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        outtakeClawClose.setName("closeClawAction");
        outtakeClawClose.setDependentActions(detectDistanceAction, moveToDistanceThreshold, purePursuitAction);
    }

    @Override
    protected boolean checkDoneCondition() {
        return outtakeClawClose.getIsDone();
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
            outtakeClawClose.updateCheckDone();

            if (detectDistanceAction.getIsDone()) {
                purePursuitAction.finishedMoving();
                Log.d("cancelPurePursuit", "Wall " + revDistance.getDistance(DistanceUnit.MM));
            }


        }
    }
}
