package com.kalipsorobotics.actions.intake;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;

public class IntakeDoorAutoAction extends KServoAutoAction {
    public IntakeDoorAutoAction(Intake intake, double targetPos) {
        super(intake.getDoorServo(), targetPos);
    }
}
