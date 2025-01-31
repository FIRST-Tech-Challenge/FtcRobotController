package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WallPickupDistanceSensorAction extends Action {

    Rev2mDistanceSensor revDistance;
    CloseWhenDetectDistanceAction closeWhenDetectAction;
    PurePursuitAction purePursuitAction;
    KServoAutoAction closeClawAction;

    public WallPickupDistanceSensorAction(Outtake outtake, Rev2mDistanceSensor revDistance, PurePursuitAction purePursuitAction) {
        this.revDistance = revDistance;
        closeWhenDetectAction = new CloseWhenDetectDistanceAction(revDistance, 44);
        closeWhenDetectAction.setName("closeWhenDetectAction");

        this.purePursuitAction = purePursuitAction;

        closeClawAction = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        closeClawAction.setName("closeClawAction");
        closeClawAction.setDependentActions(closeWhenDetectAction);
    }

    @Override
    protected boolean checkDoneCondition() {
        return closeClawAction.getIsDone();
    }

    @Override
    protected void update() {
        if (purePursuitAction.getHasStarted()) {

            closeWhenDetectAction.updateCheckDone();
            closeClawAction.updateCheckDone();
            if (!closeWhenDetectAction.getIsDone()) {
                purePursuitAction.updateCheckDone();
            } else {
                purePursuitAction.setIsDone(true);
            }

        }
    }
}