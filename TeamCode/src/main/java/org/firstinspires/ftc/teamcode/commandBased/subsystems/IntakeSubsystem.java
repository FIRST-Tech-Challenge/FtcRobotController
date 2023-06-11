package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intake;

    public IntakeSubsystem(HardwareMap hwMap) {
        intake = new CRServo(hwMap, "intake");
        intake.setInverted(false);
    }

    public void setPower(double power) {
        intake.set(power);
    }

    public double getPower() {
        return intake.get();
    }
}
