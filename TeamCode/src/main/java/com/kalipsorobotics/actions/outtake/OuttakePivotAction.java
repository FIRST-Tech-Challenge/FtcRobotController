package com.kalipsorobotics.actions.outtake;

import android.util.Log;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class OuttakePivotAction extends KActionSet {
    public OuttakePivotAction(Outtake outtake, double targetPos){
        double pivotPos = outtake.getOuttakePivotServo().getServo().getPosition();

        KServoAutoAction pivotOuttake = new KServoAutoAction(outtake.getOuttakePivotServo(), targetPos);
        pivotOuttake.setName("pivotOuttake");
        this.addAction(pivotOuttake);

        if ((pivotPos > 0.64 && targetPos < 0.64) ||
        (pivotPos < 0.375 && targetPos > 0.64)) {

            Log.d("outtakepivot", "targetPos  " + targetPos + "  pivotPos  " + pivotPos);

            KServoAutoAction pivotOuttakeMiddle = new KServoAutoAction(outtake.getOuttakePivotServo(), 0.65);
            pivotOuttakeMiddle.setName("pivotOuttakeMiddle");
            this.addAction(pivotOuttakeMiddle);
            pivotOuttake.setDependentActions(pivotOuttakeMiddle);
        }
    }
}

