package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor LEFT, RIGHT;

    private boolean isOn = false;

    public IntakeSubsystem(final DcMotor left, final DcMotor right) {
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
        if (isOn) {
            LEFT.setPower(1);
            RIGHT.setPower(-1);
        } else {
            LEFT.setPower(0);
            RIGHT.setPower(0);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        switchMotors();
    }
}
