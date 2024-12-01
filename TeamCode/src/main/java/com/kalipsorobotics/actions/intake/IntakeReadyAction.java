package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.modules.Intake;

public class IntakeReadyAction extends Action {
    Intake intake;
    IntakePivotAutoAction intakePivotAutoAction;
    IntakeDoorAutoAction intakeDoorAutoAction;
    public IntakeReadyAction() {

    }
    @Override
    public boolean checkDoneCondition() {
        return false;
    }
}
