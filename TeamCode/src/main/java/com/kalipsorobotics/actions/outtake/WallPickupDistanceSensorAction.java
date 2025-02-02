package com.kalipsorobotics.actions.outtake;

import android.util.Log;

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

    public WallPickupDistanceSensorAction(Outtake outtake, PurePursuitAction purePursuitAction) {
        this.revDistance = outtake.revDistanceClaw;
        detectDistanceAction = new DistanceDetectionAction(revDistance, 48);
        detectDistanceAction.setName("closeWhenDetectAction");

        this.purePursuitAction = purePursuitAction;

        outtakeClawClose = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        outtakeClawClose.setName("closeClawAction");
        outtakeClawClose.setDependentActions(detectDistanceAction);
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
            detectDistanceAction.updateCheckDone();
            outtakeClawClose.updateCheckDone();
            if (!detectDistanceAction.getIsDone()) {
                purePursuitAction.updateCheckDone();
                if (purePursuitAction.getIsDone()) {
                    detectDistanceAction.setIsDone(isDone);
                    Log.d("distanceForceStop", "Wall Distance:" + revDistance.getDistance(DistanceUnit.MM) + "Pos: " + SharedData.getOdometryPosition().toString());
                }
            } else {
                purePursuitAction.finishedMoving();
                Log.d("cancelPurePursuit", "Wall " + revDistance.getDistance(DistanceUnit.MM));
            }

        }
    }
}
