package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class HangSpecimenReady extends KActionSet {

    public HangSpecimenReady(Outtake outtake) {
        KServoAutoAction outtakePivotActionOpenHalf = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_HALF_POS);
        outtakePivotActionOpenHalf.setName("outtakePivotActionOpenHalf");
        this.addAction(outtakePivotActionOpenHalf);

        KServoAutoAction outtakePivotActionOpen = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_OUT_POS);
        outtakePivotActionOpen.setName("outtakePivotActionOpen");
        this.addAction(outtakePivotActionOpen);
        outtakePivotActionOpen.setDependantActions(outtakePivotActionOpenHalf);

        MoveLSAction raiseSlides = new MoveLSAction(outtake, 400);
        raiseSlides.setName("raiseSlides");
        this.addAction(raiseSlides);

    }


}
