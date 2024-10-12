package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    private CRServo intakePower;
    private Servo intakePivot;
    private Servo intakeExt;

    public enum IntakeState {
        HOME,
        COLLECT,
        STORE
    }

    public Intake(HardwareMap hMap) {

        this.intakePivot = hMap.get(Servo.class, "IntakePivot");
        this.intakePower = hMap.get(CRServo.class, "IntakePower");
        this.intakeExt = hMap.get(Servo.class, "IntakeExt");

    }

    public void extend() {

        intakeExt.setPosition(1);

    }
    public void retract() {

        intakeExt.setPosition(0);

    }

    public void setPivot(IntakeState state) {

        switch(state) {
            case HOME:
                intakePivot.setPosition(0);
                break;
            case STORE:
                intakePivot.setPosition(0.5);
                break;
            case COLLECT:
                intakePivot.setPosition(1);
                break;
        }

    }

}
