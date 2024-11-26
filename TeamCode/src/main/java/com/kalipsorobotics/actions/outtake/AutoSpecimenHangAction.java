package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.MoveLSAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.modules.Outtake;

public class AutoSpecimenHangAction extends Action {

    MoveLSAction moveLSUp;
    WaitAction waitAction;
    MoveLSAction moveDown;

    public AutoSpecimenHangAction(Outtake outtake) {
        moveLSUp = new MoveLSAction(30, outtake);

        waitAction = new WaitAction(2);
        waitAction.setDependentAction(moveLSUp);

        moveDown = new MoveLSAction(-12, outtake, 0.01);
        moveDown.setDependentAction(waitAction);
    }

    @Override
    public boolean checkDoneCondition() {
        if(moveDown.getIsDone()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        moveLSUp.updateCheckDone();
        waitAction.updateCheckDone();
        moveDown.updateCheckDone();
    }
}
