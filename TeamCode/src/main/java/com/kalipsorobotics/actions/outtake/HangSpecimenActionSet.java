package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;

public class HangSpecimenActionSet extends ActionSet {

    public HangSpecimenActionSet(Outtake outtake) {
        KServoAutoAction outtakePivotActionOpen = new KServoAutoAction(outtake.getOuttakePivotServo(), 0.0);
        this.scheduleParallel(outtakePivotActionOpen);
        //Linear slide  TODO
        KServoAutoAction outtakePivotActionClose = new KServoAutoAction(outtake.getOuttakePivotServo(), 0.925);
        this.scheduleSequential(outtakePivotActionClose);

    }


}
