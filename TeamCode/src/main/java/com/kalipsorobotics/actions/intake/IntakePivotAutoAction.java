package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;

public class IntakePivotAutoAction extends KServoAutoAction {
    public IntakePivotAutoAction(Intake intake, double targetPos) {
        super(intake.getIntakePivotServo(), targetPos);
    }
}
