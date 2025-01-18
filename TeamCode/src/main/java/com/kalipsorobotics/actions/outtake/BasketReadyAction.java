package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class BasketReadyAction extends KActionSet {
    public BasketReadyAction(Outtake outtake) {

        MoveLSAction raiseSlidesBasket = new MoveLSAction(outtake, Outtake.LS_SAMPLE_BASKET_READY_POS);
        raiseSlidesBasket.setName("raiseSlidesBasket");
        this.addAction(raiseSlidesBasket);

        KServoAutoAction pivotOuttakeHalfway = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_HALFWAY_BASKET_POS);
        pivotOuttakeHalfway.setName("pivotOuttakeHalfway");
        this.addAction(pivotOuttakeHalfway);

        KServoAutoAction pivotOuttakeBasket = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_BASKET_POS);
        pivotOuttakeBasket.setName("pivotOuttakeBasket");
        pivotOuttakeBasket.setDependentActions(raiseSlidesBasket, pivotOuttakeHalfway);
        this.addAction(pivotOuttakeBasket);

    }
}
