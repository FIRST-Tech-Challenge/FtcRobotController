package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;

public class AutoSampleDumpAction extends Action {

    MoveLSAction moveLSToBasket;
    OuttakePivotAutoAction pivotOut;
    WaitAction waitAfterDrop;
    OuttakeClawAutoAction clawOpen;
    MoveLSAction moveLSToZero;

    public AutoSampleDumpAction(Outtake outtake) {
        this.dependentAction = new DoneStateAction();

        moveLSToBasket = new MoveLSAction(CalculateTickPer.mmToTicksLS(1473.2), outtake);
        pivotOut = new OuttakePivotAutoAction(outtake, OuttakePivotAutoAction.Position.BASKET);
        pivotOut.setDependentAction(moveLSToBasket);
        clawOpen = new OuttakeClawAutoAction(outtake, OuttakeClawAutoAction.ClawPosition.OPEN);
        clawOpen.setDependentAction(pivotOut);
        waitAfterDrop = new WaitAction(300);
        waitAfterDrop.setDependentAction(clawOpen);
        moveLSToZero = new MoveLSAction(0, outtake);
        moveLSToZero.setDependentAction(waitAfterDrop);
    }

    @Override
    public boolean checkDoneCondition() {
        if(moveLSToZero.getIsDone()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        moveLSToBasket.updateCheckDone();
        pivotOut.updateCheckDone();
        clawOpen.updateCheckDone();
        waitAfterDrop.updateCheckDone();
        moveLSToZero.updateCheckDone();
    }
}
