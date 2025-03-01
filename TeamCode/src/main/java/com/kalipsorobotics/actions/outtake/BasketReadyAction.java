package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class BasketReadyAction extends KActionSet {

    public BasketReadyAction(Outtake outtake) {
        this(outtake, Outtake.OUTTAKE_PIVOT_BASKET_POS);
    }

    public BasketReadyAction(Outtake outtake, double outtakePivotPos) {

        MoveLSAction raiseSlidesBasket = new MoveLSAction(outtake, Outtake.LS_SAMPLE_BASKET_READY_POS);
        raiseSlidesBasket.setName("raiseSlidesBasket");
        this.addAction(raiseSlidesBasket);

        KServoAutoAction pivotOuttakeHalfway = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_HALFWAY_BASKET_POS);
        pivotOuttakeHalfway.setName("pivotOuttakeHalfway");
        this.addAction(pivotOuttakeHalfway);

        CheckLSPastHeightMM checkLSPastBasketReadyPos = new CheckLSPastHeightMM(raiseSlidesBasket,
                Outtake.LS_SAMPLE_BASKET_READY_POS);
        checkLSPastBasketReadyPos.setName("checkLSPastBasketReadyPos");
        this.addAction(checkLSPastBasketReadyPos);

        KServoAutoAction pivotOuttakeBasket = new KServoAutoAction(outtake.getOuttakePivotServo(), outtakePivotPos);
        pivotOuttakeBasket.setName("pivotOuttakeBasket");
        pivotOuttakeBasket.setDependentActions(checkLSPastBasketReadyPos, pivotOuttakeHalfway);
        this.addAction(pivotOuttakeBasket);

    }
}
