package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BarHangDistanceSensorAction extends Action {

    Rev2mDistanceSensor revDistance;
    DistanceDetectionAction closeWhenDetectAction;
    PurePursuitAction purePursuitAction;
    KServoAutoAction detectDistanceAction;

    public BarHangDistanceSensorAction(Outtake outtake, Rev2mDistanceSensor revDistance, PurePursuitAction purePursuitAction) {
        this.revDistance = revDistance;
        closeWhenDetectAction = new DistanceDetectionAction(revDistance, 155); //145
        closeWhenDetectAction.setName("closeWhenDetectAction");

        this.purePursuitAction = purePursuitAction;

        detectDistanceAction = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        detectDistanceAction.setName("closeClawAction");
        detectDistanceAction.setDependentActions(closeWhenDetectAction);
    }

    @Override
    protected boolean checkDoneCondition() {
        return detectDistanceAction.getIsDone();
    }

    @Override
    protected void update() {
        if (isDone) {
            return;
        }
        if (purePursuitAction.getHasStarted()) {
            closeWhenDetectAction.updateCheckDone();
            detectDistanceAction.updateCheckDone();
            if (!closeWhenDetectAction.getIsDone()) {
                purePursuitAction.updateCheckDone();
            } else {
                purePursuitAction.finishedMoving();
                Log.d("cancelPurePursuit", "Bar " + revDistance.getDistance(DistanceUnit.MM));
            }

        }
    }
}