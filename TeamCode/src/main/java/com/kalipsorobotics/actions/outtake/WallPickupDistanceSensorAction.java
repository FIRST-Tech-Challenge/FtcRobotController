package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class WallPickupDistanceSensorAction extends Action {

    Rev2mDistanceSensor revDistance;
    CloseWhenDetectDistanceAction closeWhenDetectAction;
    PurePursuitAction purePursuitAction;
    SpecimenHang specimenHang;

    public WallPickupDistanceSensorAction(Outtake outtake, Rev2mDistanceSensor revDistance, PurePursuitAction purePursuitAction) {
        this.revDistance = revDistance;
        closeWhenDetectAction = new CloseWhenDetectDistanceAction(revDistance, 44);
        closeWhenDetectAction.setName("closeWhenDetectAction");

        this.purePursuitAction = purePursuitAction;

        specimenHang= new SpecimenHang(outtake);
        specimenHang.setName("specimenHang");
        specimenHang.setDependentActions(closeWhenDetectAction);
    }

    @Override
    protected boolean checkDoneCondition() {
        return specimenHang.getIsDone();
    }

    @Override
    protected void update() {
        if (purePursuitAction.getHasStarted()) {
            closeWhenDetectAction.updateCheckDone();
            specimenHang.updateCheckDone();
            if (!closeWhenDetectAction.getIsDone()) {
                purePursuitAction.updateCheckDone();
            } else {
                purePursuitAction.setIsDone(true);
            }

        }
    }
}