package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class HangSpecimenReady extends KActionSet {

    public HangSpecimenReady(Outtake outtake) {
        KServoAutoAction outtakePivotActionOpen = new KServoAutoAction(outtake.getOuttakePivotServo(), 0.0);
        outtakePivotActionOpen.setName("outtakePivotActionOpen");
        this.addAction(outtakePivotActionOpen);

        MoveLSAction raiseSlides = new MoveLSAction(outtake, 100);
        raiseSlides.setName("raiseSlides");
        this.addAction(raiseSlides);

    }


}
