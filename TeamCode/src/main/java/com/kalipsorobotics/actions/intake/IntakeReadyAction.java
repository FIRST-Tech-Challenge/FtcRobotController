package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.modules.Intake;

public class IntakeReadyAction extends Action {
    Intake intake;
    IntakePivotAutoAction intakePivotAutoAction;
    IntakeDoorAutoAction intakeDoorAutoAction;
    public IntakeReadyAction() {
        intakeDoorAutoAction = new IntakeDoorAutoAction(intake, 0.15);
        intakePivotAutoAction = new IntakePivotAutoAction(intake, 0.0);
    }
    @Override
    public boolean checkDoneCondition() {
        return false;
    }
}
