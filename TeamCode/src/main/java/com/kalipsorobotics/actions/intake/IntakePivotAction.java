package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.modules.Intake;

//0.75 down
//~0.45 up
public class IntakePivotAction {
    final private Intake intake;
    private final KServo intakePivotServo;

    public static final double INTAKE_PIVOT_DOWN_POS = 0.80;
    public static final double INTAKE_PIVOT_UP_POS = 0.55;

    private boolean isDown = true;

    public IntakePivotAction(Intake intake) {
        this.intake = intake;
        intakePivotServo = intake.getIntakePivotServo();
    }

    public void setPosition(double position) {
        intakePivotServo.setPosition(position);
    }

    public void moveDown() {
        setPosition(INTAKE_PIVOT_DOWN_POS);
        isDown = true;
    }

    public void moveUp() {
        setPosition(INTAKE_PIVOT_UP_POS);
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
