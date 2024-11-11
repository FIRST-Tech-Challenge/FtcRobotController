package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.Servo;
//0.6 closed
//0.245 opened
public class IntakeLinkageAction {

    private final Intake intake;
    private final Servo linkageServo1;
    private final Servo linkageServo2;


    private boolean isRetracted = true;

    public IntakeLinkageAction(Intake intake) {
        this.intake = intake;
        this.linkageServo1 = intake.getLinkageServo1();
        this.linkageServo2 = intake.getLinkageServo2();
    }

    public void moveIntakeSlide(double position) {
        linkageServo1.setPosition(position);
        linkageServo2.setPosition(position);
    }

    public void extend() {
        moveIntakeSlide(0.245);
        isRetracted = false;
    }

    public void retract() {
        moveIntakeSlide(0.6);
        isRetracted = true;
    }

    public void togglePosition() {
        if (!isRetracted) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isRetracted() {
        return isRetracted;
    }

    public Intake getIntake() {
        return intake;
    }
}
