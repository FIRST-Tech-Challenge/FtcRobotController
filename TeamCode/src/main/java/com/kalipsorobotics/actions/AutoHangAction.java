package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.hang.HangHookAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;

public class AutoHangAction extends Action {
    MoveLSAction moveLSUp;
    WaitAction waitAction;
    MoveLSAction moveLSDown;
    WaitAction waitActionTwo;
    MoveLSAction pullUp;
    HangHookAction hangHookAction;

    public AutoHangAction(Outtake outtake) {
        this.dependentActions.add(new DoneStateAction());

        moveLSUp = new MoveLSAction(outtake, CalculateTickPer.inchToTicksLS(28));

        waitAction = new WaitAction(400);
        waitAction.setDependantActions(moveLSUp);

        moveLSDown = new MoveLSAction(outtake, CalculateTickPer.inchToTicksLS(-5));
        moveLSDown.setDependantActions(waitAction);

        waitActionTwo = new WaitAction(300);
        waitActionTwo.setDependantActions(moveLSDown);

        pullUp = new MoveLSAction(outtake, CalculateTickPer.inchToTicksLS(-20));
        pullUp.setDependantActions(waitActionTwo);

        hangHookAction = new HangHookAction(outtake);
        hangHookAction.setDependantActions(waitActionTwo);
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
