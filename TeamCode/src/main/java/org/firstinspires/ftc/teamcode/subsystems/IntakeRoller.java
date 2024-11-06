package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeRoller extends SubsystemBase {

    private CRServo intakePower;
    private States state = States.STOP;
    public enum States {
        OUTTAKE,
        INTAKE,
        STOP
    }

    @Override
    public void periodic() {
        switch (state) {
            case STOP:
                intakePower.setPower(0);
                break;
            case INTAKE:
                intakePower.setPower(1);
                break;
            case OUTTAKE:
                intakePower.setPower(-1);
                break;
        }
    }

    public IntakeRoller(HardwareMap hMap) {
        this.intakePower = hMap.get(CRServo.class, "IntakePower");
    }

    public States getState() {
        return state;
    }

    public void setState(States state) {
        this.state = state;
    }

    public void rollerIntake() {
        intakePower.setPower(1);
    }

    public void rollerStop() {
        intakePower.setPower(0);
    }

    public void rollerOuttake() {
        intakePower.setPower(-1);
    }

}
