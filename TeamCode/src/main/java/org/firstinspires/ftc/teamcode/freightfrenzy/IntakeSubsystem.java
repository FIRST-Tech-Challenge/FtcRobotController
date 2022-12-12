package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase{
    private final MotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "intake");
    }

    public void forward() {
        motor.set(1);
    }

    public void reverse() {
        motor.set(-1);
    }

    public void stop() {
        motor.set(0);
    }
}