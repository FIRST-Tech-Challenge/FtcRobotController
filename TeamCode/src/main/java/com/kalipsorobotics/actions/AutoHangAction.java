package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.hang.HangHookAction;
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
    HangHookAction hangHookAction;

    public AutoHangAction(Outtake outtake) {
        this.dependentAction = new DoneStateAction();

        moveLSUp = new MoveLSAction(CalculateTickInches.inchToTicksLS(28), outtake);

        waitAction = new WaitAction(4);
        waitAction.setDependentAction(moveLSUp);

        moveLSDown = new MoveLSAction(CalculateTickInches.inchToTicksLS(-5), outtake);
        moveLSDown.setDependentAction(waitAction);

        waitActionTwo = new WaitAction(3);
        waitActionTwo.setDependentAction(moveLSDown);

        pullUp = new MoveLSAction(CalculateTickInches.inchToTicksLS(-20), outtake, 1);
        pullUp.setDependentAction(waitActionTwo);

        hangHookAction = new HangHookAction(outtake);
        hangHookAction.setDependentAction(waitActionTwo);
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
        hangHookAction.updateCheckDone();
    }
}
