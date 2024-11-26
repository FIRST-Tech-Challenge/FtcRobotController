package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSubsystem extends SubsystemBase {

    DcMotorEx powerMotor;
    ServoEx tilterServo;
    boolean currentState = false;

    public IntakeSubsystem(DcMotorEx powerMotor, ServoEx tilterServo) {
        this.powerMotor = powerMotor;
        this.tilterServo = tilterServo;
        tilterServo.setRange(0, 45);
    }

    public void tiltIntake() {
        tilterServo.setPosition(1);
    }

    public void untiltIntake() {
        tilterServo.setPosition(0);
    }

    public void setIntakeState(boolean activated) {
        powerMotor.setPower(activated ? 1 : 0);
        currentState = activated;
    }

    public void toggleIntakeState() {
        setIntakeState(!currentState);
    }

}
