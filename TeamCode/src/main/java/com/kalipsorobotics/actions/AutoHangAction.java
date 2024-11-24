package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.outtake.AutoBasketAction;
import com.kalipsorobotics.actions.outtake.AutoSpecimenHangAction;
import com.kalipsorobotics.math.CalculateTickInches;
import com.kalipsorobotics.modules.Outtake;

public class AutoHangAction extends Action {
    MoveLSAction moveLSUp;
    WaitAction waitAction;
    MoveLSAction moveLSDown;
    WaitAction waitActionTwo;
    MoveLSAction pullUp;

    public AutoHangAction(Outtake outtake) {
        moveLSUp = new MoveLSAction(CalculateTickInches.inchToTicksLS(28), outtake);

        waitAction = new WaitAction(4);
        waitAction.setDependentAction(moveLSUp);

        moveLSDown = new MoveLSAction(CalculateTickInches.inchToTicksLS(-5), outtake);
        moveLSDown.setDependentAction(waitAction);

        waitActionTwo = new WaitAction(3);
        waitActionTwo.setDependentAction(moveLSDown);

        pullUp = new MoveLSAction(CalculateTickInches.inchToTicksLS(-10), outtake, 0.5);
        pullUp.setDependentAction(waitActionTwo);
    }

    @Override
    public boolean checkDoneCondition() {
        return false;
    }

    @Override
    public void update() {
        moveLSUp.updateCheckDone();
        waitAction.updateCheckDone();
        moveLSDown.updateCheckDone();
        waitActionTwo.updateCheckDone();
        pullUp.updateCheckDone();
    }
}
