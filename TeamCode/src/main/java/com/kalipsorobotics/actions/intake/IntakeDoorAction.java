package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.modules.Intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeDoorAction {

    private Intake intake;
    private Servo intakeDoor;

    public IntakeDoorAction(Intake intake) {
        this.intake = intake;
        this.intakeDoor = intake.getSampleDoor();
    }

    public void open() {
        intakeDoor.setPosition(0);
    }

    public void close() {
        intakeDoor.setPosition(0);
    }

    public Intake getIntake() {
        return intake;
    }
    public Servo getIntakeDoor() {
        return intakeDoor;
    }
}
