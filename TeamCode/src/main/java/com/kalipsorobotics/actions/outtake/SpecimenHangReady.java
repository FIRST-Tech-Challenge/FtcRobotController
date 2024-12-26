package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class SpecimenHangReady extends KActionSet {

    public SpecimenHangReady(Outtake outtake) {

        KServoAutoAction outtakeClawActionClose = new KServoAutoAction(outtake.getOuttakeClaw(),
                Outtake.OUTTAKE_CLAW_CLOSE);
        outtakeClawActionClose.setName("outtakeClawActionClose");
        this.addAction(outtakeClawActionClose);

//        KServoAutoAction outtakePivotActionOpenHalf = new KServoAutoAction(outtake.getOuttakePivotServo(),
//                OuttakePivotAction.OUTTAKE_PIVOT_HALF_POS);
//        outtakePivotActionOpenHalf.setName("outtakePivotActionOpenHalf");
//        this.addAction(outtakePivotActionOpenHalf);

        KServoAutoAction outtakePivotActionOpen = new KServoAutoAction(outtake.getOuttakePivotServo(),
                Outtake.OUTTAKE_PIVOT_SPECIMAN_HANG_READY_POS);
        outtakePivotActionOpen.setName("outtakePivotActionOpen");
        this.addAction(outtakePivotActionOpen);
        //outtakePivotActionOpen.setDependantActions(outtakePivotActionOpenHalf);

        MoveOuttakeLSAction raiseSlides = new MoveOuttakeLSAction(outtake, Outtake.LS_SPECIMAN_HANG_READY_MM); //450
        raiseSlides.setName("raiseSlides");
        this.addAction(raiseSlides);

    }


}
