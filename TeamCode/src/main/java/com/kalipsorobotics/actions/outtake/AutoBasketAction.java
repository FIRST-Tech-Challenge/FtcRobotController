package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.actions.MoveLSAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.math.CalculateTickInches;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.ams.AMSColorSensor;

public class AutoBasketAction extends Action {

    MoveLSAction moveLSToBasket;
    OuttakePivotAutoAction pivotOut;
    WaitAction waitAfterDrop;
    OuttakeClawAutoAction clawOpen;
    MoveLSAction moveLSToZero;

    public AutoBasketAction(Outtake outtake) {
        this.dependentAction = new DoneStateAction();

        moveLSToBasket = new MoveLSAction(CalculateTickInches.inchToTicksLS(58), outtake);
        pivotOut = new OuttakePivotAutoAction(outtake, OuttakePivotAutoAction.Position.BASKET);
        pivotOut.setDependentAction(moveLSToBasket);
        clawOpen = new OuttakeClawAutoAction(outtake, OuttakeClawAutoAction.ClawPosition.OPEN);
        clawOpen.setDependentAction(pivotOut);
        waitAfterDrop = new WaitAction(0.5);
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
