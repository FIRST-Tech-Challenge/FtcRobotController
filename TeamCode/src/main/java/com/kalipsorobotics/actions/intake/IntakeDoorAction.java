package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeDoorAction {

    private Intake intake;
    private Servo intakeDoor;

    public void IntakeDoorAction(Intake intake) {
        this.intake = intake;
        this.intakeDoor = intake.getSampleDoor();
    }

    public void open() {
        intakeDoor.setPosition();
    }

    public void close() {
        intakeDoor.setPosition();
    }

    public Intake getIntake() {
        return intake;
    }
    public Servo getIntakeDoor() {
        return intakeDoor;
    }
}
