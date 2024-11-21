package com.kalipsorobotics.actions.intake;

import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.modules.Intake;

//0.75 down
//~0.45 up
public class IntakePivotAction {
    final private Intake intake;
    private final Servo intakePivotServo;

    private boolean isDown = true;

    public IntakePivotAction(Intake intake) {
        this.intake = intake;
        intakePivotServo = intake.getIntakePivotServo();
    }

    public void setPosition(double position) {
        intakePivotServo.setPosition(position);
    }

    public void moveDown() {
        setPosition(0.8);
        isDown = true;
    }

    public void moveUp() {
        setPosition(0.55);
        isDown = false;
    }

    public void togglePosition() {
        if (!isDown) {
        moveDown();
        } else {
        moveUp();
        }
    }

    public Intake getIntake() {
        return intake;
    }
}
