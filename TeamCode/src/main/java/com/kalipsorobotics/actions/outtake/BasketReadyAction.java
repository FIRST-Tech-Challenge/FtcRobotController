package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class BasketReadyAction extends KActionSet {
    public BasketReadyAction(Outtake outtake) {
        KServoAutoAction outtakeClawActionClose = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_CLOSE_POS);
        outtakeClawActionClose.setName("outtakeClawActionClose");
        this.addAction(outtakeClawActionClose);

        MoveLSAction raiseSlides = new MoveLSAction(outtake, 1093);
        raiseSlides.setName("raiseSlides");
        this.addAction(raiseSlides);

        KServoAutoAction outtakePivotActionOpen = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_BASKET_POS);
        outtakePivotActionOpen.setName("outtakePivotActionOpen");
        this.addAction(outtakePivotActionOpen);
    }
}
