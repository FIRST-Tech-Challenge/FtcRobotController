package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final Servo LEFT, RIGHT;

    private boolean isOn = false;
    //private final int

    public IntakeSubsystem(final Servo left, final Servo right) {
        LEFT = left;
        RIGHT = right;
    }

    public void turnOn() {
        isOn = true;
    }

    public void turnOff() {
        isOn = false;
    }

    public void switchMotors() {
        /*
        if (isOn) {
            LEFT.setPower(1);
            RIGHT.setPower(-1);
        } else {
            LEFT.setPower(0);
            RIGHT.setPower(0);
        }

         */
    }

    @Override
    public void periodic() {
        super.periodic();
        switchMotors();
    }
}
