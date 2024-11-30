package com.kalipsorobotics.actions.sequences;

import android.os.SystemClock;

import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.modules.Intake;

public class IntakeSequence {
    IntakePivotAction intakePivotAction;
    IntakeLinkageAction intakeLinkageAction;
    double timeBefore;

    public IntakeSequence(IntakePivotAction intakePivotAction, IntakeLinkageAction intakeLinkageAction) {
        this.intakePivotAction = intakePivotAction;
        this.intakeLinkageAction = intakeLinkageAction;
        timeBefore = SystemClock.currentThreadTimeMillis();
    }
    public void setUpCheckDone() {
        double timeNow = SystemClock.currentThreadTimeMillis();
        timeBefore = timeNow;
    }
    public boolean checkDone(double waitLength) {
        double timeHere = SystemClock.currentThreadTimeMillis();
        if (timeHere - timeBefore > waitLength) {
            return true;
        } else {
            return false;
        }
    }
    public void extend() {
        intakeLinkageAction.extend();
        intakePivotAction.moveDown();
    }
    public void retract() {
        intakeLinkageAction.retract();
        intakePivotAction.togglePosition();
    }
}
