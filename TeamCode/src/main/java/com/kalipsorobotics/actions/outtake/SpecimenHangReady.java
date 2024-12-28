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

        MoveOuttakeLSAction raiseSlides = new MoveOuttakeLSAction(outtake, Outtake.LS_SPECIMAN_HANG_READY_MM); //450
        raiseSlides.setName("raiseSlides");
        this.addAction(raiseSlides);

        KServoAutoAction outtakePivotActionOpen = new KServoAutoAction(outtake.getOuttakePivotServo(),
                Outtake.OUTTAKE_PIVOT_SPECIMEN_HANG_READY_POS);
        outtakePivotActionOpen.setName("outtakePivotActionOpen");
        this.addAction(outtakePivotActionOpen);

    }


}
