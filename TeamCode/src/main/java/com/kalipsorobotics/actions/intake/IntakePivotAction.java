package com.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.modules.Intake;

public class IntakePivotAction {
    Intake intake;

    Servo intakePivotServo;

    public IntakePivotAction(Intake intake) {
        this.intake = intake;
        intakePivotServo = intake.getPivotServo();
    }

    public void goToPosition(double position) {
        intakePivotServo.setPosition(position);
    }
}
