package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class OuttakePivotAction extends KActionSet {
    public OuttakePivotAction(Outtake outtake, double targetPos){
        if (outtake.outtakePivotServo.getCurrentPosition() >= 0.65 && targetPos < 0.65) {
            KServoAutoAction pivotOuttakeMiddle = new KServoAutoAction(outtake.getOuttakePivotServo(), 0.65);
            pivotOuttakeMiddle.setName("pivotOuttakeMiddle");
            this.addAction(pivotOuttakeMiddle);
        }

        else {
            KServoAutoAction pivotOuttake = new KServoAutoAction(outtake.getOuttakePivotServo(), targetPos);
            pivotOuttake.setName("pivotOuttake");
            this.addAction(pivotOuttake);
        }
    }
}

