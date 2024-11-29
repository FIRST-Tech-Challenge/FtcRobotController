package com.kalipsorobotics.actions.sequences;

import android.os.SystemClock;

import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.modules.Intake;

public class IntakeSequence {
    IntakePivotAction intakePivotAction;
    IntakeLinkageAction intakeLinkageAction;

    public IntakeSequence(IntakePivotAction intakePivotAction, IntakeLinkageAction intakeLinkageAction) {
        this.intakePivotAction = intakePivotAction;
        this.intakeLinkageAction = intakeLinkageAction;
    }
    public void extend() {
        intakeLinkageAction.extend();
        SystemClock.sleep(1500);
        intakePivotAction.moveDown();
    }
    public void retract() {
        intakeLinkageAction.retract();
        intakePivotAction.togglePosition();
    }
}
