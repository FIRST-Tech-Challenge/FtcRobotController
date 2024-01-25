package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Intake;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo LEFT, RIGHT;

    private boolean isOn = false;
    //private final int

    public IntakeSubsystem(final HashMap<Intake, CRServo> INTAKE) {
        this.LEFT = INTAKE.get(Intake.LEFT);
        this.RIGHT = INTAKE.get(Intake.RIGHT);
    }

    public void turnOn() {
        isOn = true;
    }

    public void turnOff() {
        isOn = false;
    }

    public void switchMotors() {
        if (isOn) {
            LEFT.setPower(-1);
            RIGHT.setPower(1);
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
