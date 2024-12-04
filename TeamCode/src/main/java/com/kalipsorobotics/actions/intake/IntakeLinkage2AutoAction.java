package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;

public class IntakeLinkage2AutoAction extends KServoAutoAction {

    public IntakeLinkage2AutoAction(Intake intake, double targetPos) {
        super(intake.getLinkageServo2(), targetPos);
    }
}
