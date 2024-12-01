package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;

public class IntakeLinkage1AutoAction extends KServoAutoAction {

    public IntakeLinkage1AutoAction(Intake intake, double targetPos) {
        super(intake.getLinkageServo1(), targetPos);
    }
}
